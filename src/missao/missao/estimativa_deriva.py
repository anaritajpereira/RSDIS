"""Estima a posição e a deriva do alvo a partir das deteções do drone."""
from collections import deque
import math

from missao.estimativa import ajustar_deriva
from missao_interfaces.msg import DetecaoAlvo, EstimativaAlvo
import rclpy
from rclpy.node import Node


def segundos(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class EstimativaDeriva(Node):
    """Regressão linear das deteções numa janela de tempo, sem os outliers."""

    def __init__(self):
        super().__init__('estimativa_deriva')
        parametros = {
            'janela': 20.0,  # segundos de deteções usados no ajuste
            'pontos_min': 10,
            'duracao_min': 2.0,  # com menos tempo que isto a velocidade é só ruído
            # uma deteção nova a mais disto da reta atual é deitada fora
            'distancia_outlier': 6.0,
            # na segunda passagem do ajuste tiram-se os pontos a mais disto da reta
            'limite_residuo': 3.0,
            # se rejeitarmos muitas seguidas é porque a reta é que está mal: recomeça
            'rejeicoes_para_recomecar': 15,
            'confianca_min': 0.3,
            # depois disto sem deteções deixa de publicar (quem usa extrapola)
            'validade': 5.0,
            'frequencia': 5.0,
            'frame_mundo': 'oceano_corrente',
        }
        for nome, valor in parametros.items():
            self.declare_parameter(nome, valor)

        self.amostras = deque()
        self.ajuste = None
        self.rejeicoes_seguidas = 0
        self.instante_ultima_detecao = None
        self.a_publicar = False

        self.create_subscription(
            DetecaoAlvo, '/alvo/detecao', self.ao_receber_detecao, 50)
        self.pub_estimativa = self.create_publisher(
            EstimativaAlvo, '/alvo/estimativa', 10)
        self.create_timer(1.0 / self.param('frequencia'), self.ciclo)
        self.get_logger().info('À espera de deteções em /alvo/detecao')

    def param(self, nome):
        return self.get_parameter(nome).value

    def previsao(self, instante):
        """Posição prevista pelo ajuste atual nesse instante."""
        x0, y0, vx, vy, _ = self.ajuste
        return x0 + vx * instante, y0 + vy * instante

    def ao_receber_detecao(self, msg):
        if msg.confianca < self.param('confianca_min'):
            return
        instante = segundos(msg.header.stamp)

        if self.ajuste is not None:
            previsto_x, previsto_y = self.previsao(instante)
            erro = math.hypot(msg.x - previsto_x, msg.y - previsto_y)
            if erro > self.param('distancia_outlier'):
                self.rejeicoes_seguidas += 1
                self.get_logger().info(
                    f'Deteção rejeitada: a {erro:.1f} m do previsto',
                    throttle_duration_sec=2.0)
                if self.rejeicoes_seguidas < self.param('rejeicoes_para_recomecar'):
                    return
                self.get_logger().warning(
                    'Muitas deteções rejeitadas seguidas; a recomeçar o ajuste')
                self.amostras.clear()
                self.ajuste = None
        self.rejeicoes_seguidas = 0

        self.amostras.append((instante, msg.x, msg.y))
        self.instante_ultima_detecao = instante
        # esquece o que já saiu da janela
        while self.amostras and self.amostras[0][0] < instante - self.param('janela'):
            self.amostras.popleft()

    def ciclo(self):
        if not self.amostras:
            return
        duracao = self.amostras[-1][0] - self.amostras[0][0]
        if duracao < self.param('duracao_min'):
            return
        ajuste = ajustar_deriva(list(self.amostras), self.param('limite_residuo'),
                                self.param('pontos_min'))
        if ajuste is None:
            return
        self.ajuste = ajuste

        agora = self.get_clock().now()
        idade = segundos(agora.to_msg()) - self.instante_ultima_detecao
        if idade > self.param('validade'):
            if self.a_publicar:
                self.a_publicar = False
                self.get_logger().info('Sem deteções recentes; deixo de publicar')
            return
        if not self.a_publicar:
            self.a_publicar = True
            self.get_logger().info('A publicar estimativas em /alvo/estimativa')

        # publica a posição prevista para "agora", não para a última deteção
        x, y = self.previsao(segundos(agora.to_msg()))
        estimativa = EstimativaAlvo()
        estimativa.header.stamp = agora.to_msg()
        estimativa.header.frame_id = self.param('frame_mundo')
        estimativa.x, estimativa.y = x, y
        estimativa.vx, estimativa.vy = ajuste[2], ajuste[3]
        self.pub_estimativa.publish(estimativa)
        self.get_logger().info(
            f'Alvo em ({x:.1f}, {y:.1f}), deriva ({ajuste[2]:.2f}, {ajuste[3]:.2f}) m/s '
            f'com {ajuste[4]}/{len(self.amostras)} pontos', throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = EstimativaDeriva()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
