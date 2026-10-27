"""Máquina de estados do drone (fase 1: ir a uma coordenada e regressar)."""
import math

from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from missao.geometria import (mundo_para_corpo, normalizar_angulo, saturar,
                              saturar_vetor, yaw_de_quaternion)
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

        drone = self.p('drone')
        self.pub_cmd = self.create_publisher(Twist, f'/{drone}/cmd_vel', 10)
        self.pub_estado = self.create_publisher(String, f'/{drone}/estado', 10)
        self.create_subscription(
            PoseStamped, f'/{drone}/pose_mundo', self.ao_receber_pose, 10)
        self.create_subscription(
            PointStamped, f'/{drone}/objetivo', self.ao_receber_objetivo, 10)

        self.pose = None
        self.t_pose = None
        self.objetivo = None
        self.t_chegada = None
        self.estado = POUSADO
        self.create_timer(1.0 / self.p('frequencia'), self.ciclo)
        self.get_logger().info(f'Pronto; objetivos em /{drone}/objetivo')

    def p(self, nome):
        return self.get_parameter(nome).value

    def mudar_estado(self, novo):
        self.get_logger().info(f'{self.estado} -> {novo}')
        self.estado = novo
        self.t_chegada = None

    def ao_receber_pose(self, msg):
        self.pose = msg.pose
        self.t_pose = self.get_clock().now()

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
        idade = (self.get_clock().now() - self.t_pose).nanoseconds * 1e-9
        return idade < self.p('timeout_pose')

    def ciclo(self):
        self.pub_estado.publish(String(data=self.estado))
        if not self.pose_valida():
            self.get_logger().warning('Sem pose recente do drone',
                                      throttle_duration_sec=5.0)
            self.publicar(0.0, 0.0, 0.0, 0.0)
            return

        pos = self.pose.position
        o = self.pose.orientation
        yaw = yaw_de_quaternion(o.x, o.y, o.z, o.w)
        altitude = self.p('altitude')

        if self.estado == POUSADO:
            self.publicar(0.0, 0.0, 0.0, 0.0)

        elif self.estado == DESCOLAR:
            vz = saturar(self.p('ganho_altitude') * (altitude - pos.z),
                         self.p('velocidade_vertical'))
            self.publicar(0.0, 0.0, vz, 0.0)
            if abs(altitude - pos.z) < self.p('tolerancia_altitude'):
                self.mudar_estado(IR)

        elif self.estado == IR:
            if self.voar_para(self.objetivo, altitude, pos, yaw):
                agora = self.get_clock().now()
                if self.t_chegada is None:
                    self.t_chegada = agora
                    self.get_logger().info('Objetivo atingido; a pairar')
                pairado = (agora - self.t_chegada).nanoseconds * 1e-9
                if pairado >= self.p('tempo_pairar'):
                    self.mudar_estado(REGRESSAR)

        elif self.estado == REGRESSAR:
            base = (self.p('base_x'), self.p('base_y'))
            if self.voar_para(base, altitude, pos, yaw):
                self.mudar_estado(ATERRAR)

        elif self.estado == ATERRAR:
            vx, vy = self.velocidade_horizontal(
                (self.p('base_x'), self.p('base_y')), pos)
            self.publicar(vx, vy, -self.p('velocidade_aterrar'), 0.0, yaw)
            if pos.z < self.p('altura_base') + self.p('margem_pouso'):
                self.publicar(0.0, 0.0, 0.0, 0.0)
                self.mudar_estado(POUSADO)

    def velocidade_horizontal(self, alvo, pos):
        k = self.p('ganho_posicao')
        return saturar_vetor(k * (alvo[0] - pos.x), k * (alvo[1] - pos.y),
                             self.p('velocidade_cruzeiro'))

    def voar_para(self, alvo, altitude, pos, yaw):
        """Comanda um passo em direção ao alvo; devolve True se já chegou."""
        dx, dy = alvo[0] - pos.x, alvo[1] - pos.y
        distancia = math.hypot(dx, dy)
        vx, vy = self.velocidade_horizontal(alvo, pos)
        vz = saturar(self.p('ganho_altitude') * (altitude - pos.z),
                     self.p('velocidade_vertical'))

        # Perto do alvo o rumo pedido oscila muito; mantém-se o atual.
        wz = 0.0
        if distancia > self.p('distancia_rumo'):
            erro = normalizar_angulo(math.atan2(dy, dx) - yaw)
            wz = saturar(self.p('ganho_rumo') * erro, self.p('rotacao_max'))

        self.publicar(vx, vy, vz, wz, yaw)
        return (distancia < self.p('tolerancia_posicao')
                and abs(altitude - pos.z) < self.p('tolerancia_altitude'))

    def publicar(self, vx, vy, vz, wz, yaw=0.0):
        """Publica uma velocidade dada no referencial do mundo."""
        if self.p('cmd_vel_referencial') == 'corpo':
            vx, vy = mundo_para_corpo(vx, vy, yaw)
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z = vx, vy, vz
        cmd.angular.z = wz
        self.pub_cmd.publish(cmd)


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
            node.publicar(0.0, 0.0, 0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
