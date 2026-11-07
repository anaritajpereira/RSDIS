"""Máquina de estados do drone (fase 1: ir a uma coordenada e regressar)."""
import math

from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from missao.geometria import (mundo_para_corpo, normalizar_angulo, saturar,
                              velocidade_desejada, yaw_de_quaternion)
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

POUSADO = 'POUSADO'
DESCOLAR = 'DESCOLAR'
IR = 'IR'
REGRESSAR = 'REGRESSAR'
ATERRAR = 'ATERRAR'


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
            'cmd_vel_referencial': 'corpo',
            'timeout_pose': 1.0,
            'frequencia': 20.0,
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

        self.pose = None
        self.instante_pose = None
        self.objetivo = None
        self.instante_chegada = None
        self.ja_voou = False
        self.estado = POUSADO
        self.create_timer(1.0 / self.param('frequencia'), self.ciclo)
        self.get_logger().info(f'Pronto; objetivos em /{drone}/objetivo')

    def param(self, nome):
        return self.get_parameter(nome).value

    def mudar_estado(self, novo_estado):
        self.get_logger().info(f'{self.estado} -> {novo_estado}')
        self.estado = novo_estado
        self.instante_chegada = None

    def ao_receber_pose(self, msg):
        self.pose = msg.pose
        self.instante_pose = self.get_clock().now()

    def ao_receber_objetivo(self, msg):
        self.objetivo = (msg.point.x, msg.point.y)
        self.get_logger().info(
            f'Novo objetivo: ({msg.point.x:.1f}, {msg.point.y:.1f})')
        if self.estado == POUSADO:
            self.mudar_estado(DESCOLAR)
        elif self.estado != DESCOLAR:
            self.mudar_estado(IR)

    def pose_valida(self):
        if self.pose is None:
            return False
        idade_pose = (self.get_clock().now()
                      - self.instante_pose).nanoseconds * 1e-9
        return idade_pose < self.param('timeout_pose')

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

        if self.estado == POUSADO:
            # Com os motores já ligados, velocidade zero deixa-o pairar e subir aos poucos.
            vel_vertical = -self.param('velocidade_aterrar') if self.ja_voou else 0.0
            self.publicar_velocidade(0.0, 0.0, vel_vertical, 0.0)

        elif self.estado == DESCOLAR:
            self.ja_voou = True
            vel_vertical = saturar(
                self.param('ganho_altitude') * (altitude - posicao.z),
                self.param('velocidade_vertical'))
            self.publicar_velocidade(0.0, 0.0, vel_vertical, 0.0)
            if abs(altitude - posicao.z) < self.param('tolerancia_altitude'):
                self.mudar_estado(IR)

        elif self.estado == IR:
            if self.voar_para(self.objetivo, altitude, posicao, rumo):
                agora = self.get_clock().now()
                if self.instante_chegada is None:
                    self.instante_chegada = agora
                    self.get_logger().info('Objetivo atingido; a pairar')
                tempo_a_pairar = (agora - self.instante_chegada).nanoseconds * 1e-9
                if tempo_a_pairar >= self.param('tempo_pairar'):
                    self.mudar_estado(REGRESSAR)

        elif self.estado == REGRESSAR:
            base = (self.param('base_x'), self.param('base_y'))
            if self.voar_para(base, altitude, posicao, rumo):
                self.mudar_estado(ATERRAR)

        elif self.estado == ATERRAR:
            vel_x, vel_y = self.velocidade_horizontal(
                (self.param('base_x'), self.param('base_y')), posicao)
            self.publicar_velocidade(
                vel_x, vel_y, -self.param('velocidade_aterrar'), 0.0, rumo)
            if posicao.z < self.param('altura_base') + self.param('margem_pouso'):
                self.mudar_estado(POUSADO)

    def velocidade_horizontal(self, destino, posicao):
        return velocidade_desejada(posicao.x, posicao.y, destino[0], destino[1],
                                   self.param('ganho_posicao'),
                                   self.param('velocidade_cruzeiro'))

    def voar_para(self, destino, altitude, posicao, rumo):
        """Comanda um passo em direção ao destino; devolve True se já chegou."""
        falta_x, falta_y = destino[0] - posicao.x, destino[1] - posicao.y
        distancia = math.hypot(falta_x, falta_y)
        vel_x, vel_y = self.velocidade_horizontal(destino, posicao)
        vel_vertical = saturar(
            self.param('ganho_altitude') * (altitude - posicao.z),
            self.param('velocidade_vertical'))

        # Perto do destino o rumo pedido oscila muito; mantém-se o atual.
        vel_rotacao = 0.0
        if distancia > self.param('distancia_rumo'):
            erro_rumo = normalizar_angulo(math.atan2(falta_y, falta_x) - rumo)
            vel_rotacao = saturar(self.param('ganho_rumo') * erro_rumo,
                                  self.param('rotacao_max'))

        self.publicar_velocidade(vel_x, vel_y, vel_vertical, vel_rotacao, rumo)
        return (distancia < self.param('tolerancia_posicao')
                and abs(altitude - posicao.z) < self.param('tolerancia_altitude'))

    def publicar_velocidade(self, vel_x, vel_y, vel_vertical, vel_rotacao,
                            rumo=0.0):
        """Publica uma velocidade dada no referencial do mundo."""
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
        # Com SIGTERM o contexto pode já estar fechado.
        if rclpy.ok():
            node.publicar_velocidade(0.0, 0.0, 0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
