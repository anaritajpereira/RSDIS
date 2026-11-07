"""Máquina de estados do drone: busca em faixas, acompanhamento do alvo e regresso."""
import math

from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from missao.geometria import (bateria_necessaria, espacamento_faixas,
                              mundo_para_corpo, normalizar_angulo,
                              ponto_a_distancia, pontos_busca, saturar,
                              velocidade_desejada, yaw_de_quaternion)
from missao_interfaces.msg import DetecaoAlvo
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

POUSADO = 'POUSADO'
DESCOLAR = 'DESCOLAR'
IR = 'IR'
BUSCAR = 'BUSCAR'
ACOMPANHAR = 'ACOMPANHAR'
REGRESSAR = 'REGRESSAR'
ATERRAR = 'ATERRAR'

# só nestes estados é que se verifica a bateria (em REGRESSAR/ATERRAR já vai para casa)
ESTADOS_EM_VOO = (DESCOLAR, IR, BUSCAR, ACOMPANHAR)


def segundos(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class DroneMissao(Node):
    """Controla o drone por /<drone>/cmd_vel a partir da pose do Gazebo."""

    def __init__(self):
        super().__init__('drone_missao')
        parametros = {
            'drone': 'drone',
            'altitude': 20.0,
            'velocidade_cruzeiro': 4.0,
            'velocidade_vertical': 1.5,
            'velocidade_aterrar': 0.5,
            'ganho_posicao': 0.5,
            'ganho_altitude': 0.8,
            'ganho_rumo': 1.0,
            'rotacao_max': 1.0,
            'distancia_rumo': 5.0,
            'tolerancia_posicao': 1.0,
            'tolerancia_altitude': 0.5,
            'tempo_pairar': 5.0,
            'base_x': 0.0,
            'base_y': 0.0,
            'altura_base': 1.0,
            'margem_pouso': 0.3,
            # o cmd_vel do MBZIRC é no referencial do drone (testámos), não do mundo
            'cmd_vel_referencial': 'corpo',
            'timeout_pose': 1.0,
            'frequencia': 20.0,
            # busca: se busca_automatica estiver a true descola logo e começa as faixas
            'busca_automatica': True,
            'area_x_min': -120.0,
            'area_x_max': 120.0,
            'area_y_min': -120.0,
            'area_y_max': 120.0,
            'fov_horizontal': 1.0472,  # 60 graus, do sdf da câmara
            'depressao_min': 0.2,  # só conta a água até ~11 graus abaixo do horizonte
            'sobreposicao': 0.2,
            'tolerancia_ponto_busca': 5.0,
            # a câmara é frontal, por isso não dá para ficar por cima do alvo;
            # fica a esta distância e virado para ele
            'distancia_acompanhamento': 60.0,
            'confianca_min': 0.3,
            'tempo_perda_alvo': 3.0,
            # bateria: ver verificar_bateria()
            'bateria_minima': 0.15,
            'margem_bateria': 0.05,
            'autonomia': 1800.0,  # na sim gasta 100% em ~30 min (medido)
        }
        for nome, valor in parametros.items():
            self.declare_parameter(nome, valor)

        drone = self.param('drone')
        self.pub_velocidade = self.create_publisher(
            Twist, f'/{drone}/cmd_vel', 10)
        self.pub_estado = self.create_publisher(String, f'/{drone}/estado', 10)
        self.create_subscription(
            PoseStamped, f'/{drone}/pose_mundo', self.ao_receber_pose, 10)
        self.create_subscription(
            PointStamped, f'/{drone}/objetivo', self.ao_receber_objetivo, 10)
        self.create_subscription(
            DetecaoAlvo, '/alvo/detecao', self.ao_receber_detecao, 10)
        self.create_subscription(
            BatteryState, f'/model/{drone}/battery/linear_battery/state',
            self.ao_receber_bateria, 10)

        self.pose = None
        self.instante_pose = None
        self.objetivo = None
        self.instante_chegada = None
        self.ja_voou = False
        self.missao_iniciada = False
        self.estado_apos_descolar = BUSCAR
        self.estado = POUSADO

        self.pontos_busca = []
        self.indice_busca = 0
        self.pontos_visitados = 0
        self.alvo = None
        self.instante_detecao = None

        self.bateria = None
        self.instante_bateria = None
        self.primeira_leitura_bateria = None
        self.bateria_baixa = False

        self.create_timer(1.0 / self.param('frequencia'), self.ciclo)
        self.get_logger().info(f'Pronto; objetivos manuais em /{drone}/objetivo')

    def param(self, nome):
        return self.get_parameter(nome).value

    def agora(self):
        return self.get_clock().now()

    def mudar_estado(self, novo_estado):
        self.get_logger().info(f'{self.estado} -> {novo_estado}')
        self.estado = novo_estado
        self.instante_chegada = None

    def ao_receber_pose(self, msg):
        self.pose = msg.pose
        self.instante_pose = self.agora()

    def ao_receber_objetivo(self, msg):
        if self.bateria_baixa:
            self.get_logger().warning('Objetivo ignorado: bateria baixa')
            return
        self.objetivo = (msg.point.x, msg.point.y)
        self.get_logger().info(
            f'Novo objetivo: ({msg.point.x:.1f}, {msg.point.y:.1f})')
        if self.estado == POUSADO:
            self.estado_apos_descolar = IR
            self.missao_iniciada = True
            self.mudar_estado(DESCOLAR)
        elif self.estado == DESCOLAR:
            self.estado_apos_descolar = IR
        else:
            self.mudar_estado(IR)

    def ao_receber_detecao(self, msg):
        if msg.confianca < self.param('confianca_min'):
            return
        self.alvo = (msg.x, msg.y)
        self.instante_detecao = self.agora()

    def ao_receber_bateria(self, msg):
        if msg.capacity <= 0.0 or math.isnan(msg.charge):
            return
        # o percentage vem de 0 a 100 pela ponte, mais simples fazer carga/capacidade
        self.bateria = msg.charge / msg.capacity
        instante = segundos(msg.header.stamp)
        if self.primeira_leitura_bateria is None:
            self.primeira_leitura_bateria = (instante, self.bateria)
        self.instante_bateria = instante

    def consumo_por_segundo(self):
        """Consumo medido desde a primeira leitura, nunca abaixo do nominal."""
        nominal = 1.0 / self.param('autonomia')
        instante_inicial, bateria_inicial = self.primeira_leitura_bateria
        decorrido = self.instante_bateria - instante_inicial
        # nos primeiros segundos a estimativa ainda é muito instável
        if decorrido < 20.0:
            return nominal
        return max(nominal, (bateria_inicial - self.bateria) / decorrido)

    def pose_valida(self):
        if self.pose is None:
            return False
        idade_pose = (self.agora() - self.instante_pose).nanoseconds * 1e-9
        return idade_pose < self.param('timeout_pose')

    def alvo_recente(self):
        if self.instante_detecao is None:
            return False
        idade = (self.agora() - self.instante_detecao).nanoseconds * 1e-9
        return idade < self.param('tempo_perda_alvo')

    def ciclo(self):
        self.pub_estado.publish(String(data=self.estado))
        if not self.pose_valida():
            self.get_logger().warning('Sem pose recente do drone',
                                      throttle_duration_sec=5.0)
            self.publicar_velocidade(0.0, 0.0, 0.0, 0.0)
            return

        posicao = self.pose.position
        orientacao = self.pose.orientation
        rumo = yaw_de_quaternion(orientacao.x, orientacao.y, orientacao.z,
                                 orientacao.w)
        altitude = self.param('altitude')
        base = (self.param('base_x'), self.param('base_y'))

        if self.estado in ESTADOS_EM_VOO:
            self.verificar_bateria(posicao, base)

        if self.estado == POUSADO:
            # depois de aterrar tem de continuar a empurrar para baixo, senão
            # com velocidade 0 os motores ainda o levantam aos poucos
            vel_vertical = -self.param('velocidade_aterrar') if self.ja_voou else 0.0
            self.publicar_velocidade(0.0, 0.0, vel_vertical, 0.0)
            if self.param('busca_automatica') and not self.missao_iniciada:
                self.missao_iniciada = True
                self.estado_apos_descolar = BUSCAR
                self.mudar_estado(DESCOLAR)

        elif self.estado == DESCOLAR:
            # sobe na vertical até à altitude e só depois passa ao estado seguinte
            self.ja_voou = True
            vel_vertical = saturar(
                self.param('ganho_altitude') * (altitude - posicao.z),
                self.param('velocidade_vertical'))
            self.publicar_velocidade(0.0, 0.0, vel_vertical, 0.0)
            if abs(altitude - posicao.z) < self.param('tolerancia_altitude'):
                if self.estado_apos_descolar == BUSCAR:
                    self.preparar_busca(posicao)
                self.mudar_estado(self.estado_apos_descolar)

        elif self.estado == IR:
            # modo de teste da fase 1: vai ao ponto, paira um bocado e volta
            if self.voar_para(self.objetivo, altitude, posicao, rumo):
                agora = self.agora()
                if self.instante_chegada is None:
                    self.instante_chegada = agora
                    self.get_logger().info('Objetivo atingido; a pairar')
                tempo_a_pairar = (agora - self.instante_chegada).nanoseconds * 1e-9
                if tempo_a_pairar >= self.param('tempo_pairar'):
                    self.mudar_estado(REGRESSAR)

        elif self.estado == BUSCAR:
            if self.alvo_recente():
                self.get_logger().info(
                    f'Alvo detetado em ({self.alvo[0]:.1f}, {self.alvo[1]:.1f})')
                self.mudar_estado(ACOMPANHAR)
                return
            self.seguir_busca(altitude, posicao, rumo)

        elif self.estado == ACOMPANHAR:
            if not self.alvo_recente():
                self.get_logger().info('Alvo perdido; a retomar a busca')
                # retoma na faixa mais perto em vez de voltar à primeira
                self.indice_busca = self.inicio_de_faixa_mais_proximo(posicao)
                self.pontos_visitados = 0
                self.mudar_estado(BUSCAR)
                return
            # ponto a 60 m do alvo do lado onde o drone já está, assim não dá voltas
            destino = ponto_a_distancia(
                self.alvo[0], self.alvo[1], posicao.x, posicao.y,
                self.param('distancia_acompanhamento'))
            rumo_para_alvo = math.atan2(self.alvo[1] - posicao.y,
                                        self.alvo[0] - posicao.x)
            self.voar_para(destino, altitude, posicao, rumo, rumo_para_alvo)

        elif self.estado == REGRESSAR:
            if self.voar_para(base, altitude, posicao, rumo):
                self.mudar_estado(ATERRAR)

        elif self.estado == ATERRAR:
            # continua a corrigir x/y enquanto desce para não sair da plataforma
            vel_x, vel_y = self.velocidade_horizontal(base, posicao)
            self.publicar_velocidade(
                vel_x, vel_y, -self.param('velocidade_aterrar'), 0.0, rumo)
            if posicao.z < self.param('altura_base') + self.param('margem_pouso'):
                self.mudar_estado(POUSADO)

    def preparar_busca(self, posicao):
        espacamento = espacamento_faixas(
            self.param('altitude'), self.param('fov_horizontal'),
            self.param('depressao_min'), self.param('sobreposicao'))
        self.pontos_busca = pontos_busca(
            self.param('area_x_min'), self.param('area_x_max'),
            self.param('area_y_min'), self.param('area_y_max'), espacamento)
        self.indice_busca = self.inicio_de_faixa_mais_proximo(posicao)
        self.pontos_visitados = 0
        self.get_logger().info(
            f'Busca: {len(self.pontos_busca) // 2} faixas espaçadas de '
            f'{espacamento:.0f} m')

    def inicio_de_faixa_mais_proximo(self, posicao):
        """Índice do início de faixa mais próximo, para varrer faixas inteiras."""
        inicios = range(0, len(self.pontos_busca), 2)
        return min(inicios, key=lambda i: math.hypot(
            self.pontos_busca[i][0] - posicao.x,
            self.pontos_busca[i][1] - posicao.y))

    def seguir_busca(self, altitude, posicao, rumo):
        destino = self.pontos_busca[self.indice_busca]
        self.voar_para(destino, altitude, posicao, rumo)
        distancia = math.hypot(destino[0] - posicao.x, destino[1] - posicao.y)
        if distancia < self.param('tolerancia_ponto_busca'):
            self.indice_busca = (self.indice_busca + 1) % len(self.pontos_busca)
            self.pontos_visitados += 1
            # deu a volta a todos os pontos e não encontrou nada, recomeça
            if self.pontos_visitados % len(self.pontos_busca) == 0:
                self.get_logger().info('Área toda varrida; a recomeçar a busca')

    def verificar_bateria(self, posicao, base):
        if self.bateria is None:
            return
        distancia_base = math.hypot(base[0] - posicao.x, base[1] - posicao.y)
        necessaria = bateria_necessaria(
            distancia_base, posicao.z - self.param('altura_base'),
            self.param('velocidade_cruzeiro'), self.param('velocidade_aterrar'),
            self.consumo_por_segundo())
        # volta quando a bateria chega ao maior destes dois: o mínimo fixo (15%)
        # ou o que precisa para voltar mais uma margem
        limite = max(self.param('bateria_minima'),
                     necessaria + self.param('margem_bateria'))
        if self.bateria <= limite:
            self.bateria_baixa = True
            self.get_logger().warning(
                f'Bateria a {self.bateria:.0%} (limite {limite:.0%}, '
                f'precisa de {necessaria:.0%} para voltar): a regressar')
            self.mudar_estado(REGRESSAR)

    def velocidade_horizontal(self, destino, posicao):
        return velocidade_desejada(posicao.x, posicao.y, destino[0], destino[1],
                                   self.param('ganho_posicao'),
                                   self.param('velocidade_cruzeiro'))

    def voar_para(self, destino, altitude, posicao, rumo, rumo_desejado=None):
        """
        Comanda um passo em direção ao destino; devolve True se já chegou.

        Sem rumo_desejado, o drone vira-se para onde vai.
        """
        falta_x, falta_y = destino[0] - posicao.x, destino[1] - posicao.y
        distancia = math.hypot(falta_x, falta_y)
        vel_x, vel_y = self.velocidade_horizontal(destino, posicao)
        vel_vertical = saturar(
            self.param('ganho_altitude') * (altitude - posicao.z),
            self.param('velocidade_vertical'))

        # muito perto do destino o atan2 salta de um lado para o outro e o drone
        # ficava a rodar, por isso aí deixa-se o rumo como está
        if rumo_desejado is None and distancia > self.param('distancia_rumo'):
            rumo_desejado = math.atan2(falta_y, falta_x)
        vel_rotacao = 0.0
        if rumo_desejado is not None:
            erro_rumo = normalizar_angulo(rumo_desejado - rumo)
            vel_rotacao = saturar(self.param('ganho_rumo') * erro_rumo,
                                  self.param('rotacao_max'))

        self.publicar_velocidade(vel_x, vel_y, vel_vertical, vel_rotacao, rumo)
        return (distancia < self.param('tolerancia_posicao')
                and abs(altitude - posicao.z) < self.param('tolerancia_altitude'))

    def publicar_velocidade(self, vel_x, vel_y, vel_vertical, vel_rotacao,
                            rumo=0.0):
        """Publica uma velocidade dada no referencial do mundo."""
        # as contas são todas feitas no mundo, só aqui é que se passa para o corpo
        if self.param('cmd_vel_referencial') == 'corpo':
            vel_x, vel_y = mundo_para_corpo(vel_x, vel_y, rumo)
        comando = Twist()
        comando.linear.x, comando.linear.y = vel_x, vel_y
        comando.linear.z = vel_vertical
        comando.angular.z = vel_rotacao
        self.pub_velocidade.publish(comando)


def main():
    rclpy.init()
    node = DroneMissao()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # com SIGTERM (ctrl+c no launch) o rclpy às vezes já fechou e dava erro
        if rclpy.ok():
            node.publicar_velocidade(0.0, 0.0, 0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
