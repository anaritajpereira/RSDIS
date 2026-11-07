"""Máquina de estados do barco robô (fase 2: seguir a última posição do alvo)."""
import math

from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from missao.geometria import (comando_vetorial, ponto_a_distancia,
                              velocidade_desejada, yaw_de_quaternion)
from missao_interfaces.msg import DetecaoAlvo
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

AGUARDAR = 'AGUARDAR'
NAVEGAR = 'NAVEGAR'
CHEGOU = 'CHEGOU'


class BarcoMissao(Node):
    """Leva o barco robô para junto do alvo (ou de um ponto) por /<barco>/cmd_vel."""

    def __init__(self):
        super().__init__('barco_missao')
        parametros = {
            'barco': 'barco_1',
            'velocidade_max': 2.5,
            'ganho_velocidade': 0.15,
            'ganho_correcao': 1.0,
            'filtro_velocidade': 0.2,  # peso da medida nova (0 a 1)
            'ganho_rumo': 1.0,
            'rotacao_max': 0.8,
            'erro_rumo_max': 0.8,
            'velocidade_minima': 0.2,
            'tolerancia_chegada': 5.0,
            'histerese': 5.0,  # para não andar sempre a trocar NAVEGAR/CHEGOU
            'timeout_pose': 1.0,
            'frequencia': 10.0,
            # o barco_2 tem uns 22 m de comprimento, 20 m do centro chega para não bater
            'seguir_alvo': True,
            'distancia_seguranca': 20.0,
            'confianca_min': 0.3,
        }
        for nome, valor in parametros.items():
            self.declare_parameter(nome, valor)

        barco = self.param('barco')
        self.pub_velocidade = self.create_publisher(
            Twist, f'/{barco}/cmd_vel', 10)
        self.pub_estado = self.create_publisher(String, f'/{barco}/estado', 10)
        self.create_subscription(
            PoseStamped, f'/{barco}/pose_mundo', self.ao_receber_pose, 10)
        self.create_subscription(
            PointStamped, f'/{barco}/objetivo', self.ao_receber_objetivo, 10)
        self.create_subscription(
            DetecaoAlvo, '/alvo/detecao', self.ao_receber_detecao, 10)

        self.pose = None
        self.instante_pose = None
        self.pose_anterior = None
        self.velocidade_medida = (0.0, 0.0)
        self.objetivo = None
        self.alvo = None
        self.estado = AGUARDAR
        self.create_timer(1.0 / self.param('frequencia'), self.ciclo)
        self.get_logger().info(
            f'Pronto; segue /alvo/detecao ou objetivos em /{barco}/objetivo')

    def param(self, nome):
        return self.get_parameter(nome).value

    def mudar_estado(self, novo_estado):
        self.get_logger().info(f'{self.estado} -> {novo_estado}')
        self.estado = novo_estado

    def ao_receber_pose(self, msg):
        self.pose = msg.pose
        self.instante_pose = self.get_clock().now()

        # o WAM-V escorrega muito de lado, por isso estimamos a velocidade real a
        # partir das poses (diferenças com um filtro passa-baixo simples)
        instante = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
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
        # objetivo manual ganha ao alvo até chegar outra deteção (útil para testes)
        self.alvo = None
        self.objetivo = (msg.point.x, msg.point.y)
        self.get_logger().info(
            f'Novo objetivo: ({msg.point.x:.1f}, {msg.point.y:.1f})')
        if self.estado != NAVEGAR:
            self.mudar_estado(NAVEGAR)

    def ao_receber_detecao(self, msg):
        if not self.param('seguir_alvo') or msg.confianca < self.param('confianca_min'):
            return
        if self.alvo is None:
            self.get_logger().info(f'A seguir o alvo em ({msg.x:.1f}, {msg.y:.1f})')
            if self.estado != NAVEGAR:
                self.mudar_estado(NAVEGAR)
        self.alvo = (msg.x, msg.y)

    def pose_valida(self):
        if self.pose is None:
            return False
        idade_pose = (self.get_clock().now()
                      - self.instante_pose).nanoseconds * 1e-9
        return idade_pose < self.param('timeout_pose')

    def ciclo(self):
        self.pub_estado.publish(String(data=self.estado))
        # o alvo está sempre a mexer-se, por isso o objetivo é recalculado a cada ciclo
        if self.pose is not None and self.alvo is not None:
            self.objetivo = ponto_a_distancia(
                self.alvo[0], self.alvo[1], self.pose.position.x,
                self.pose.position.y, self.param('distancia_seguranca'))
        if self.objetivo is None or not self.pose_valida():
            if self.objetivo is not None:
                self.get_logger().warning('Sem pose recente do barco',
                                          throttle_duration_sec=5.0)
            self.publicar_velocidade(0.0, 0.0)
            return

        posicao = self.pose.position
        orientacao = self.pose.orientation
        objetivo_x, objetivo_y = self.objetivo
        distancia = math.hypot(objetivo_x - posicao.x, objetivo_y - posicao.y)

        if self.estado == NAVEGAR and distancia < self.param('tolerancia_chegada'):
            self.mudar_estado(CHEGOU)
        elif (self.estado == CHEGOU and distancia
              > self.param('tolerancia_chegada') + self.param('histerese')):
            self.mudar_estado(NAVEGAR)

        # o controlo é igual em NAVEGAR e CHEGOU; em CHEGOU se parássemos os
        # motores a corrente levava o barco
        desejada_x, desejada_y = velocidade_desejada(
            posicao.x, posicao.y, objetivo_x, objetivo_y,
            self.param('ganho_velocidade'), self.param('velocidade_max'))
        # em vez de apontar só para o objetivo, aponta para vd + k*(vd - v).
        # Assim compensa o escorregar de lado e a corrente. Só com controlo de
        # rumo o barco ficava às voltas ao objetivo (foi o que aconteceu ao início).
        ganho = self.param('ganho_correcao')
        medida_x, medida_y = self.velocidade_medida
        avanco, rotacao = comando_vetorial(
            yaw_de_quaternion(orientacao.x, orientacao.y, orientacao.z,
                              orientacao.w),
            desejada_x + ganho * (desejada_x - medida_x),
            desejada_y + ganho * (desejada_y - medida_y),
            self.param('ganho_rumo'), self.param('velocidade_max'),
            self.param('rotacao_max'), self.param('erro_rumo_max'),
            self.param('velocidade_minima'))
        self.publicar_velocidade(avanco, rotacao)

        self.get_logger().info(
            f'{self.estado}: distância {distancia:.1f} m',
            throttle_duration_sec=5.0)

    def publicar_velocidade(self, avanco, rotacao):
        comando = Twist()
        comando.linear.x = avanco
        comando.angular.z = rotacao
        self.pub_velocidade.publish(comando)


def main():
    rclpy.init()
    node = BarcoMissao()
    try:
        rclpy.spin(node)
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
