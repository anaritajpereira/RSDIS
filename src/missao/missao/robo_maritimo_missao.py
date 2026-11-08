"""Robô marítimo (o WAM-V): servidor das ações Resgatar e Rebocar."""
import math
import time

from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from missao import gazebo
from missao.geometria import (comando_vetorial, normalizar_angulo,
                              ponto_a_distancia, saturar, tempo_intercecao,
                              velocidade_com_avanco, velocidade_desejada,
                              yaw_de_quaternion)
from missao_interfaces.action import Rebocar, Resgatar
from missao_interfaces.msg import EstimativaAlvo
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

AGUARDAR = 'AGUARDAR'
# resgate (fase 3)
INTERCETAR = 'INTERCETAR'
APROXIMAR = 'APROXIMAR'
CONFIRMAR = 'CONFIRMAR'
ACOMPANHAR_ALVO = 'ACOMPANHAR_ALVO'
# reboque (fase 4)
ENGATAR = 'ENGATAR'
REBOCAR = 'REBOCAR'
ENTREGUE = 'ENTREGUE'
NAVEGAR = 'NAVEGAR'  # só para testes, com objetivos manuais

NOME_BARRA = 'barra_reboque'


def segundos(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class RoboMaritimoMissao(Node):
    """Leva o robô marítimo até ao barco à deriva e, se pedirem, reboca-o."""

    def __init__(self):
        super().__init__('robo_maritimo_missao')
        parametros = {
            # nome usado nos nossos tópicos (/robo_maritimo/...)
            'nome': 'robo_maritimo',
            # estes dois vêm da simulação e não os podemos mudar: no Gazebo o
            # robô chama-se barco_1 e o barco_teleop.py só ouve este cmd_vel
            'modelo_gazebo': 'barco_1',
            'topico_cmd_vel': '/barco_1/cmd_vel',
            # 'intercetar' vai ao ponto onde o alvo vai estar; 'perseguir' vai
            # sempre à posição atual (para comparar os dois)
            'modo_intercecao': 'intercetar',
            'velocidade_max': 2.5,
            # perto do alvo anda mais devagar, senão chega lançado e fica muito perto
            'velocidade_max_perto': 1.5,
            'ganho_velocidade': 0.15,
            'ganho_correcao': 1.0,
            'filtro_velocidade': 0.2,  # peso da medida nova (0 a 1)
            'ganho_rumo': 1.0,
            'rotacao_max': 0.8,
            'erro_rumo_max': 0.8,
            'velocidade_minima': 0.2,
            # o barco à deriva tem uns 22 m, 20 m do centro chega para não bater
            'distancia_seguranca': 20.0,
            # abaixo disto (ao alvo) deixa de intercetar e passa a aproximar-se
            'distancia_aproximar': 40.0,
            'tolerancia_chegada': 5.0,
            'histerese': 5.0,  # para não andar sempre a trocar de estado
            # tem de ficar este tempo junto ao alvo para contar como confirmado
            'tempo_confirmar': 5.0,
            # --- reboque ---
            # o robô engata do lado do alvo virado para a zona de entrega, de
            # proa para ela, a esta distância do centro do alvo (11 m de meio
            # navio + 2.5 m de meio WAM-V + folga)
            'distancia_engate': 16.0,
            'tolerancia_engate': 3.0,
            'erro_rumo_engate': 0.25,  # rad
            'tempo_engate': 2.0,  # tem de estar alinhado este tempo antes de prender
            'velocidade_reboque': 0.8,
            # o barco_teleop está afinado para o WAM-V sozinho: com 2.5 m/s só
            # pede ~580 N aos motores. A rebocar pedimos até 4 m/s, que já é
            # perto da força máxima (~1440 N nos dois motores)
            'avanco_max_reboque': 4.0,
            # com o alvo preso o conjunto quase não roda (o navio tem muito
            # arrasto de rotação), por isso: corrige pouco a corrente, aceita
            # andar com o rumo um bocado torto e pede mais força para virar
            'ganho_correcao_reboque': 1.0,
            'erro_rumo_max_reboque': 1.2,  # rad; acima disto só tenta virar
            'rotacao_max_reboque': 2.0,
            'raio_entrega': 8.0,  # entregue quando o robô chega a esta distância da zona
            # se o largarmos, a corrente leva-o outra vez; por omissão fica preso
            # e o robô segura-o na zona
            'largar_na_entrega': False,
            # ao segurar, a força cai para zero quando a velocidade pedida é
            # menor do que isto (m/s): é o que evita andar sempre a fundo
            'escala_forca_reboque': 0.5,
            # para o DetachableJoint
            'mundo': 'oceano_corrente',
            'link_robo': 'base_link',
            'modelo_alvo': 'barco_2',  # o barco à deriva, com o nome do Gazebo
            'link_alvo': 'link',
            'timeout_pose': 1.0,
            'frequencia': 10.0,
        }
        for nome, valor in parametros.items():
            self.declare_parameter(nome, valor)

        nome = self.param('nome')
        self.topico_soltar = f'/{nome}/reboque/soltar'
        self.pub_velocidade = self.create_publisher(
            Twist, self.param('topico_cmd_vel'), 10)
        self.pub_estado = self.create_publisher(String, f'/{nome}/estado', 10)
        self.create_subscription(
            PoseStamped, f'/{nome}/pose_mundo', self.ao_receber_pose, 10)
        self.create_subscription(
            PointStamped, f'/{nome}/objetivo', self.ao_receber_objetivo, 10)
        self.create_subscription(
            EstimativaAlvo, '/alvo/estimativa', self.ao_receber_estimativa, 10)
        # a execução das ações fica num ciclo com sleep, por isso precisa de um
        # grupo reentrante e de um executor com várias threads
        grupo = ReentrantCallbackGroup()
        cancelar = lambda pedido: CancelResponse.ACCEPT  # noqa: E731
        self.servidor_resgate = ActionServer(
            self, Resgatar, f'/{nome}/resgatar', self.executar_resgate,
            goal_callback=self.ao_receber_pedido_resgate,
            cancel_callback=cancelar, callback_group=grupo)
        self.servidor_reboque = ActionServer(
            self, Rebocar, f'/{nome}/rebocar', self.executar_reboque,
            goal_callback=self.ao_receber_pedido_reboque,
            cancel_callback=cancelar, callback_group=grupo)

        self.pose = None
        self.instante_pose = None
        self.pose_anterior = None
        self.velocidade_medida = (0.0, 0.0)
        self.objetivo_manual = None
        # última estimativa do alvo: (x, y, vx, vy, instante)
        self.estimativa = None
        self.pedido_atual = None
        self.instante_junto_alvo = None
        self.distancia_alvo = float('nan')
        # reboque
        self.zona_entrega = None
        self.preso = False
        self.pronto_para_prender = False
        self.chegou_a_zona = False
        self.ponto_largada = None
        self.ponto_espera = None  # onde o robô fica a segurar o alvo entregue
        self.distancia_reboque = float('nan')

        self.estado = AGUARDAR
        self.create_timer(1.0 / self.param('frequencia'), self.ciclo)
        self.get_logger().info(
            f'Pronto; ações em /{nome}/resgatar e /{nome}/rebocar, '
            f'modo {self.param("modo_intercecao")}')

    def param(self, nome):
        return self.get_parameter(nome).value

    def agora(self):
        return segundos(self.get_clock().now().to_msg())

    def mudar_estado(self, novo_estado):
        self.get_logger().info(f'{self.estado} -> {novo_estado}')
        self.estado = novo_estado
        self.instante_junto_alvo = None

    def ao_receber_pose(self, msg):
        self.pose = msg.pose
        self.instante_pose = self.get_clock().now()

        # o WAM-V escorrega muito de lado, por isso estimamos a velocidade real a
        # partir das poses (diferenças com um filtro passa-baixo simples)
        instante = segundos(msg.header.stamp)
        x, y = msg.pose.position.x, msg.pose.position.y
        if self.pose_anterior is not None:
            x_anterior, y_anterior, instante_anterior = self.pose_anterior
            intervalo = instante - instante_anterior
            if intervalo > 0.0:
                peso_novo = self.param('filtro_velocidade')
                vel_x, vel_y = self.velocidade_medida
                self.velocidade_medida = (
                    peso_novo * (x - x_anterior) / intervalo
                    + (1 - peso_novo) * vel_x,
                    peso_novo * (y - y_anterior) / intervalo
                    + (1 - peso_novo) * vel_y)
        self.pose_anterior = (x, y, instante)

    def ao_receber_objetivo(self, msg):
        # objetivo manual (para testes) passa à frente de tudo; se houver uma
        # ação a correr, ela é abortada
        self.objetivo_manual = (msg.point.x, msg.point.y)
        self.get_logger().info(
            f'Objetivo manual: ({msg.point.x:.1f}, {msg.point.y:.1f})')
        if self.preso:
            self.largar()
        if self.estado != NAVEGAR:
            self.mudar_estado(NAVEGAR)

    def ao_receber_estimativa(self, msg):
        self.estimativa = (msg.x, msg.y, msg.vx, msg.vy, segundos(msg.header.stamp))

    def alvo_agora(self):
        """Posição e velocidade do alvo agora, extrapoladas da última estimativa."""
        if self.estimativa is None:
            return None
        x, y, vx, vy, instante = self.estimativa
        # depois do hand-off o drone vai embora e deixa de haver estimativas
        # novas; continuamos com a última deriva conhecida
        decorrido = max(0.0, self.agora() - instante)
        return x + vx * decorrido, y + vy * decorrido, vx, vy

    def rumo(self):
        orientacao = self.pose.orientation
        return yaw_de_quaternion(orientacao.x, orientacao.y, orientacao.z,
                                 orientacao.w)

    # ---------- ação Resgatar ----------

    def ao_receber_pedido_resgate(self, pedido):
        self.get_logger().info(
            f'Pedido de resgate: alvo em ({pedido.x:.1f}, {pedido.y:.1f})')
        return GoalResponse.ACCEPT

    def executar_resgate(self, pedido_ativo):
        # um pedido novo substitui o anterior (o ciclo do antigo vê e termina)
        self.pedido_atual = pedido_ativo
        if self.estimativa is None:
            # ainda sem estimativa: usa a posição do pedido, parada
            pedido = pedido_ativo.request
            self.estimativa = (pedido.x, pedido.y, 0.0, 0.0, self.agora())
        self.objetivo_manual = None
        self.mudar_estado(INTERCETAR)

        resultado = Resgatar.Result()
        while rclpy.ok():
            motivo = self.motivo_para_parar(pedido_ativo)
            if motivo:
                resultado.mensagem = motivo
                return resultado
            if self.estado == ACOMPANHAR_ALVO:
                pedido_ativo.succeed()
                resultado.sucesso = True
                resultado.mensagem = (f'Junto ao alvo a {self.distancia_alvo:.1f} m; '
                                      'a acompanhá-lo')
                return resultado

            # no primeiro instante ainda não há distância calculada
            if not math.isnan(self.distancia_alvo):
                feedback = Resgatar.Feedback()
                feedback.distancia = self.distancia_alvo
                feedback.estado = self.estado
                pedido_ativo.publish_feedback(feedback)
            time.sleep(0.5)

        pedido_ativo.abort()
        return resultado

    def motivo_para_parar(self, pedido_ativo):
        """Vê se a ação tem de acabar sem sucesso; se sim, termina-a e diz porquê."""
        if pedido_ativo.is_cancel_requested:
            pedido_ativo.canceled()
            if self.preso:
                self.largar()
            self.mudar_estado(AGUARDAR)
            return 'Cancelado'
        if self.pedido_atual is not pedido_ativo:
            pedido_ativo.abort()
            return 'Substituído por um pedido novo'
        if self.estado == NAVEGAR:
            pedido_ativo.abort()
            return 'Interrompido por um objetivo manual'
        return ''

    # ---------- ação Rebocar ----------

    def ao_receber_pedido_reboque(self, pedido):
        if self.estimativa is None:
            self.get_logger().warning('Pedido de reboque recusado: não sei onde está o alvo')
            return GoalResponse.REJECT
        self.get_logger().info(
            f'Pedido de reboque para ({pedido.x:.1f}, {pedido.y:.1f})')
        return GoalResponse.ACCEPT

    def executar_reboque(self, pedido_ativo):
        self.pedido_atual = pedido_ativo
        self.zona_entrega = (pedido_ativo.request.x, pedido_ativo.request.y)
        self.objetivo_manual = None
        self.pronto_para_prender = False
        self.chegou_a_zona = False
        self.mudar_estado(ENGATAR)

        resultado = Rebocar.Result()
        while rclpy.ok():
            motivo = self.motivo_para_parar(pedido_ativo)
            if motivo:
                resultado.mensagem = motivo
                return resultado

            # prender e soltar chamam o "ign" e demoram um bocado, por isso
            # fazem-se aqui e não no ciclo de controlo
            if self.pronto_para_prender:
                self.pronto_para_prender = False
                if not self.prender():
                    self.mudar_estado(ACOMPANHAR_ALVO)
                    pedido_ativo.abort()
                    resultado.mensagem = 'Não consegui prender o alvo no Gazebo'
                    return resultado
                self.mudar_estado(REBOCAR)
            if self.chegou_a_zona:
                self.chegou_a_zona = False
                resultado.mensagem = f'Alvo a {self.distancia_reboque:.1f} m da zona de entrega'
                if self.param('largar_na_entrega'):
                    self.largar()
                    self.ponto_largada = (self.pose.position.x, self.pose.position.y)
                    resultado.mensagem += '; largado'
                else:
                    resultado.mensagem += '; o robô fica a segurá-lo'
                self.mudar_estado(ENTREGUE)
                pedido_ativo.succeed()
                resultado.sucesso = True
                return resultado

            if not math.isnan(self.distancia_reboque):
                feedback = Rebocar.Feedback()
                feedback.distancia = self.distancia_reboque
                feedback.estado = self.estado
                pedido_ativo.publish_feedback(feedback)
            time.sleep(0.5)

        pedido_ativo.abort()
        return resultado

    def prender(self):
        """Cria a barra de reboque entre o robô e o alvo."""
        # Primeiro tentámos prender o alvo diretamente ao robô (junta fixa),
        # mas assim o conjunto fica rígido e o robô não o consegue virar: a
        # corrente a empurrar o navio de lado ganha sempre. Com uma barra com
        # articulações nas pontas, cada um roda à vontade, como com um cabo.
        alvo_x, alvo_y, _, _ = self.alvo_agora()
        posicao = self.pose.position
        comprimento = math.hypot(posicao.x - alvo_x, posicao.y - alvo_y)
        sdf = gazebo.sdf_barra_reboque(
            comprimento, self.param('modelo_gazebo'), self.param('link_robo'),
            self.param('modelo_alvo'), self.param('link_alvo'), self.topico_soltar)
        # a barra aparece a meio caminho, apontada do alvo para o robô
        criada = gazebo.criar_modelo(
            self.param('mundo'), sdf, NOME_BARRA,
            (posicao.x + alvo_x) / 2.0, (posicao.y + alvo_y) / 2.0, posicao.z,
            math.atan2(posicao.y - alvo_y, posicao.x - alvo_x))
        if not criada:
            self.get_logger().error('O Gazebo não criou a barra de reboque')
            return False
        self.preso = True
        self.get_logger().info(f'Alvo preso com uma barra de {comprimento:.1f} m; a rebocar')
        return True

    def largar(self):
        gazebo.soltar(self.topico_soltar)
        gazebo.apagar_modelo(self.param('mundo'), NOME_BARRA)
        self.preso = False
        self.get_logger().info('Alvo solto')

    # ---------- controlo ----------

    def pose_valida(self):
        if self.pose is None:
            return False
        idade_pose = (self.get_clock().now()
                      - self.instante_pose).nanoseconds * 1e-9
        return idade_pose < self.param('timeout_pose')

    def ciclo(self):
        self.pub_estado.publish(String(data=self.estado))
        if not self.pose_valida():
            if self.estado != AGUARDAR:
                self.get_logger().warning('Sem pose recente do robô',
                                          throttle_duration_sec=5.0)
            self.publicar_velocidade(0.0, 0.0)
            return

        posicao = self.pose.position
        if self.estado == ENTREGUE and self.preso:
            self.puxar(posicao, self.ponto_espera)
            return
        if self.estado in (NAVEGAR, ENTREGUE):
            # segura o ponto (objetivo manual ou onde largou o alvo)
            ponto = self.objetivo_manual if self.estado == NAVEGAR else self.ponto_largada
            self.controlar(velocidade_desejada(
                posicao.x, posicao.y, ponto[0], ponto[1],
                self.param('ganho_velocidade'), self.param('velocidade_max')))
            return
        if self.estado == REBOCAR:
            self.controlar_reboque(posicao)
            return

        alvo = self.alvo_agora()
        if self.estado == AGUARDAR or alvo is None:
            self.publicar_velocidade(0.0, 0.0)
            return

        alvo_x, alvo_y, vel_alvo_x, vel_alvo_y = alvo
        self.distancia_alvo = math.hypot(alvo_x - posicao.x, alvo_y - posicao.y)
        if self.estado == ENGATAR:
            self.controlar_engate(posicao, alvo)
            return
        self.atualizar_estado()

        if self.estado == INTERCETAR:
            desejada = self.velocidade_para_intercetar(posicao, alvo)
        else:
            # perto do alvo: ponto a distancia_seguranca do lado onde estamos,
            # e somamos a deriva do alvo para andarmos com ele
            objetivo = ponto_a_distancia(alvo_x, alvo_y, posicao.x, posicao.y,
                                         self.param('distancia_seguranca'))
            desejada = velocidade_com_avanco(
                posicao.x, posicao.y, objetivo[0], objetivo[1],
                vel_alvo_x, vel_alvo_y, self.param('ganho_velocidade'),
                self.param('velocidade_max_perto'))
        self.controlar(desejada)

        self.get_logger().info(
            f'{self.estado}: alvo a {self.distancia_alvo:.1f} m',
            throttle_duration_sec=5.0)

    def atualizar_estado(self):
        distancia_seguranca = self.param('distancia_seguranca')
        tolerancia = self.param('tolerancia_chegada')
        histerese = self.param('histerese')
        # quanto falta para o ponto a distancia_seguranca do alvo
        erro = abs(self.distancia_alvo - distancia_seguranca)

        if self.estado == INTERCETAR:
            if self.distancia_alvo < self.param('distancia_aproximar'):
                self.mudar_estado(APROXIMAR)
        elif self.estado == APROXIMAR:
            if self.distancia_alvo > self.param('distancia_aproximar') + histerese:
                self.mudar_estado(INTERCETAR)
            elif erro < tolerancia:
                self.mudar_estado(CONFIRMAR)
                self.instante_junto_alvo = self.agora()
        elif self.estado == CONFIRMAR:
            if erro > tolerancia + histerese:
                self.mudar_estado(APROXIMAR)
            elif self.agora() - self.instante_junto_alvo >= self.param('tempo_confirmar'):
                self.get_logger().info(
                    f'Alvo alcançado: a {self.distancia_alvo:.1f} m do centro')
                self.mudar_estado(ACOMPANHAR_ALVO)

    def velocidade_para_intercetar(self, posicao, alvo):
        alvo_x, alvo_y, vel_alvo_x, vel_alvo_y = alvo
        ponto_x, ponto_y = alvo_x, alvo_y
        if self.param('modo_intercecao') == 'intercetar':
            # onde é que o alvo vai estar quando lá chegarmos à velocidade máxima
            tempo = tempo_intercecao(alvo_x, alvo_y, vel_alvo_x, vel_alvo_y,
                                     posicao.x, posicao.y,
                                     self.param('velocidade_max'))
            if tempo is not None:
                ponto_x += vel_alvo_x * tempo
                ponto_y += vel_alvo_y * tempo
                self.get_logger().info(
                    f'Interceção daqui a {tempo:.0f} s em ({ponto_x:.1f}, {ponto_y:.1f})',
                    throttle_duration_sec=5.0)
        objetivo = ponto_a_distancia(ponto_x, ponto_y, posicao.x, posicao.y,
                                     self.param('distancia_seguranca'))
        return velocidade_desejada(posicao.x, posicao.y, objetivo[0], objetivo[1],
                                   self.param('ganho_velocidade'),
                                   self.param('velocidade_max'))

    def controlar_engate(self, posicao, alvo):
        """Põe o robô entre o alvo e a zona de entrega, de proa para a zona."""
        alvo_x, alvo_y, vel_alvo_x, vel_alvo_y = alvo
        zona_x, zona_y = self.zona_entrega
        ponto = ponto_a_distancia(alvo_x, alvo_y, zona_x, zona_y,
                                  self.param('distancia_engate'))
        falta = math.hypot(ponto[0] - posicao.x, ponto[1] - posicao.y)
        self.distancia_reboque = falta
        tolerancia = self.param('tolerancia_engate')

        if falta > tolerancia:
            # ainda longe: vai para o ponto de engate, a andar com o alvo
            self.instante_junto_alvo = None
            self.controlar(velocidade_com_avanco(
                posicao.x, posicao.y, ponto[0], ponto[1], vel_alvo_x, vel_alvo_y,
                self.param('ganho_velocidade'), self.param('velocidade_max_perto')))
            self.get_logger().info(f'ENGATAR: faltam {falta:.1f} m para o ponto de engate',
                                   throttle_duration_sec=5.0)
            return

        # já no sítio: roda no lugar até ficar de proa para a zona de entrega
        # (o alvo fica atrás, e assim ao puxar vem direitinho)
        erro_rumo = normalizar_angulo(
            math.atan2(zona_y - posicao.y, zona_x - posicao.x) - self.rumo())
        self.publicar_velocidade(0.0, saturar(self.param('ganho_rumo') * erro_rumo,
                                              self.param('rotacao_max')))
        if abs(erro_rumo) > self.param('erro_rumo_engate'):
            self.instante_junto_alvo = None
            return
        if self.instante_junto_alvo is None:
            self.instante_junto_alvo = self.agora()
        elif self.agora() - self.instante_junto_alvo >= self.param('tempo_engate'):
            self.instante_junto_alvo = None
            self.pronto_para_prender = True

    def controlar_reboque(self, posicao):
        """Puxa o alvo até ele chegar à zona de entrega."""
        # com a barra articulada o alvo pode estar em qualquer ângulo atrás do
        # robô e não temos sensor para o ver, por isso quem tem de chegar à
        # zona é o robô; o alvo fica atrás dele, a favor da corrente
        zona = self.zona_entrega
        self.distancia_reboque = math.hypot(zona[0] - posicao.x, zona[1] - posicao.y)
        if self.distancia_reboque < self.param('raio_entrega') and not self.chegou_a_zona:
            self.ponto_espera = zona
            self.chegou_a_zona = True
        self.puxar(posicao, zona)
        self.get_logger().info(
            f'REBOCAR: robô a {self.distancia_reboque:.1f} m da zona de entrega',
            throttle_duration_sec=5.0)

    def puxar(self, posicao, objetivo):
        """Controlo com o alvo preso: leva o conjunto (ou segura-o) no objetivo."""
        desejada_x, desejada_y = velocidade_desejada(
            posicao.x, posicao.y, objetivo[0], objetivo[1],
            self.param('ganho_velocidade'), self.param('velocidade_reboque'))
        ganho = self.param('ganho_correcao_reboque')
        medida_x, medida_y = self.velocidade_medida
        vetor_x = desejada_x + ganho * (desejada_x - medida_x)
        vetor_y = desejada_y + ganho * (desejada_y - medida_y)
        erro_rumo = normalizar_angulo(math.atan2(vetor_y, vetor_x) - self.rumo())
        # ao contrário do controlo normal, puxa-se com quase toda a força mesmo
        # com o rumo um bocado torto: se o robô parar para virar, a corrente
        # leva o conjunto. Já no sítio, a força baixa com o tamanho do vetor
        # (quando o conjunto começa a fugir, o vetor cresce e volta a puxar)
        forca = min(1.0, math.hypot(vetor_x, vetor_y) / self.param('escala_forca_reboque'))
        avanco = 0.0
        if abs(erro_rumo) < self.param('erro_rumo_max_reboque'):
            avanco = self.param('avanco_max_reboque') * math.cos(erro_rumo) * forca
        rotacao = saturar(self.param('ganho_rumo') * erro_rumo,
                          self.param('rotacao_max_reboque'))
        self.publicar_velocidade(avanco, rotacao)

    def controlar(self, desejada, ganho=None, avanco_max=None):
        """Segue a velocidade desejada (no mundo) com o controlo vetorial."""
        if ganho is None:
            ganho = self.param('ganho_correcao')
        if avanco_max is None:
            avanco_max = self.param('velocidade_max')
        desejada_x, desejada_y = desejada
        # em vez de apontar só para o objetivo, aponta para vd + k*(vd - v).
        # Assim compensa o escorregar de lado e a corrente. Só com controlo de
        # rumo o robô ficava às voltas ao objetivo (foi o que aconteceu ao início).
        medida_x, medida_y = self.velocidade_medida
        avanco, rotacao = comando_vetorial(
            self.rumo(),
            desejada_x + ganho * (desejada_x - medida_x),
            desejada_y + ganho * (desejada_y - medida_y),
            self.param('ganho_rumo'), avanco_max,
            self.param('rotacao_max'), self.param('erro_rumo_max'),
            self.param('velocidade_minima'))
        self.publicar_velocidade(avanco, rotacao)

    def publicar_velocidade(self, avanco, rotacao):
        comando = Twist()
        comando.linear.x = avanco
        comando.angular.z = rotacao
        self.pub_velocidade.publish(comando)


def main():
    rclpy.init()
    node = RoboMaritimoMissao()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # com SIGTERM (ctrl+c no launch) o rclpy às vezes já fechou e dava erro
        if rclpy.ok():
            node.publicar_velocidade(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
