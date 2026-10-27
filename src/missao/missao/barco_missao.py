"""Máquina de estados do barco robô (fase 1: ir a uma coordenada)."""
import math

from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from missao.geometria import (comando_vetorial, velocidade_desejada,
                              yaw_de_quaternion)
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

AGUARDAR = 'AGUARDAR'
NAVEGAR = 'NAVEGAR'
CHEGOU = 'CHEGOU'


class BarcoMissao(Node):
    """Leva o barco robô a um ponto através de /<barco>/cmd_vel."""

    def __init__(self):
        super().__init__('barco_missao')
        parametros = {
            'barco': 'barco_1',
            'velocidade_max': 2.5,
            'ganho_velocidade': 0.15,
            'ganho_correcao': 1.0,
            'filtro_velocidade': 0.2,
            'ganho_rumo': 1.0,
            'rotacao_max': 0.8,
            'erro_rumo_max': 0.8,
            'velocidade_minima': 0.2,
            'tolerancia_chegada': 5.0,
            'histerese': 5.0,
            'timeout_pose': 1.0,
            'frequencia': 10.0,
        }
        for nome, valor in parametros.items():
            self.declare_parameter(nome, valor)

        barco = self.p('barco')
        self.pub_cmd = self.create_publisher(Twist, f'/{barco}/cmd_vel', 10)
        self.pub_estado = self.create_publisher(String, f'/{barco}/estado', 10)
        self.create_subscription(
            PoseStamped, f'/{barco}/pose_mundo', self.ao_receber_pose, 10)
        self.create_subscription(
            PointStamped, f'/{barco}/objetivo', self.ao_receber_objetivo, 10)

        self.pose = None
        self.t_pose = None
        self.anterior = None
        self.velocidade = (0.0, 0.0)
        self.objetivo = None
        self.estado = AGUARDAR
        self.create_timer(1.0 / self.p('frequencia'), self.ciclo)
        self.get_logger().info(f'Pronto; objetivos em /{barco}/objetivo')

    def p(self, nome):
        return self.get_parameter(nome).value

    def mudar_estado(self, novo):
        self.get_logger().info(f'{self.estado} -> {novo}')
        self.estado = novo

    def ao_receber_pose(self, msg):
        self.pose = msg.pose
        self.t_pose = self.get_clock().now()

        # O casco tem muita inércia e escorrega de lado: o controlo usa a velocidade medida.
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x, y = msg.pose.position.x, msg.pose.position.y
        if self.anterior is not None:
            xa, ya, ta = self.anterior
            dt = t - ta
            if dt > 0.0:
                a = self.p('filtro_velocidade')
                vx, vy = self.velocidade
                self.velocidade = (a * (x - xa) / dt + (1 - a) * vx,
                                   a * (y - ya) / dt + (1 - a) * vy)
        self.anterior = (x, y, t)

    def ao_receber_objetivo(self, msg):
        self.objetivo = (msg.point.x, msg.point.y)
        self.get_logger().info(
            f'Novo objetivo: ({msg.point.x:.1f}, {msg.point.y:.1f})')
        if self.estado != NAVEGAR:
            self.mudar_estado(NAVEGAR)

    def pose_valida(self):
        if self.pose is None:
            return False
        idade = (self.get_clock().now() - self.t_pose).nanoseconds * 1e-9
        return idade < self.p('timeout_pose')

    def ciclo(self):
        self.pub_estado.publish(String(data=self.estado))
        if self.objetivo is None or not self.pose_valida():
            if self.objetivo is not None:
                self.get_logger().warning('Sem pose recente do barco',
                                          throttle_duration_sec=5.0)
            self.publicar(0.0, 0.0)
            return

        pos = self.pose.position
        o = self.pose.orientation
        alvo_x, alvo_y = self.objetivo
        distancia = math.hypot(alvo_x - pos.x, alvo_y - pos.y)

        if self.estado == NAVEGAR and distancia < self.p('tolerancia_chegada'):
            self.mudar_estado(CHEGOU)
        elif (self.estado == CHEGOU and distancia
              > self.p('tolerancia_chegada') + self.p('histerese')):
            self.mudar_estado(NAVEGAR)

        # O controlo é o mesmo nos dois estados: em CHEGOU continua a segurar o
        # ponto contra a corrente.
        vdx, vdy = velocidade_desejada(
            pos.x, pos.y, alvo_x, alvo_y, self.p('ganho_velocidade'),
            self.p('velocidade_max'))
        # Proa para vd + k·(vd − v): anula a derrapagem lateral e a corrente.
        k = self.p('ganho_correcao')
        vx, vy = self.velocidade
        avanco, rotacao = comando_vetorial(
            yaw_de_quaternion(o.x, o.y, o.z, o.w),
            vdx + k * (vdx - vx), vdy + k * (vdy - vy),
            self.p('ganho_rumo'), self.p('velocidade_max'),
            self.p('rotacao_max'), self.p('erro_rumo_max'),
            self.p('velocidade_minima'))
        self.publicar(avanco, rotacao)

        self.get_logger().info(
            f'{self.estado}: distância {distancia:.1f} m',
            throttle_duration_sec=5.0)

    def publicar(self, avanco, rotacao):
        cmd = Twist()
        cmd.linear.x = avanco
        cmd.angular.z = rotacao
        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = BarcoMissao()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Com SIGTERM o contexto pode já estar fechado.
        if rclpy.ok():
            node.publicar(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
