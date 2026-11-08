"""Grava as métricas da missão em CSV (uma linha por segundo e os eventos)."""
import csv
import math
import os
import shutil
import time

from geometry_msgs.msg import PoseStamped
from missao_interfaces.msg import DetecaoAlvo, EstimativaAlvo
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

COLUNAS = [
    'tempo', 'estado_missao', 'estado_drone', 'estado_robo',
    'drone_x', 'drone_y', 'drone_z', 'robo_x', 'robo_y',
    'alvo_real_x', 'alvo_real_y', 'detecao_x', 'detecao_y', 'erro_detecao',
    'estimativa_x', 'estimativa_y', 'estimativa_vx', 'estimativa_vy',
    'erro_estimativa', 'distancia_robo_alvo', 'distancia_alvo_entrega', 'bateria',
]


def segundos(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def distancia(a, b):
    if a is None or b is None:
        return float('nan')
    return math.hypot(a[0] - b[0], a[1] - b[1])


class RegistoMetricas(Node):
    """Junta poses, estados, deteções e estimativas num CSV para analisar depois."""

    def __init__(self):
        super().__init__('registo_metricas')
        self.declare_parameter('pasta', os.path.join(os.getcwd(), 'metricas'))
        self.declare_parameter('modo_intercecao', 'intercetar')
        self.declare_parameter('drone', 'drone')
        self.declare_parameter('robo_maritimo', 'robo_maritimo')
        # o painel vermelho é o que o drone vê, por isso é contra ele que medimos o erro
        self.declare_parameter('alvo_real', 'marca_vermelha')
        self.declare_parameter('periodo', 1.0)
        # zona para onde o robô reboca o alvo (fase 4)
        self.declare_parameter('zona_entrega_x', 15.0)
        self.declare_parameter('zona_entrega_y', 10.0)
        # o yaml usado nesta execução; fica uma cópia junto dos CSV
        self.declare_parameter('ficheiro_parametros', '')

        drone = self.get_parameter('drone').value
        robo = self.get_parameter('robo_maritimo').value
        alvo_real = self.get_parameter('alvo_real').value

        self.ultimos = {'estado_missao': '', 'estado_drone': '', 'estado_robo': ''}
        self.posicoes = {}
        self.detecao = None
        self.estimativa = None
        self.bateria = float('nan')
        self.instante_inicio = None
        self.zona_entrega = (self.get_parameter('zona_entrega_x').value,
                             self.get_parameter('zona_entrega_y').value)

        self.create_subscription(String, '/missao/estado',
                                 self.guardar_estado('estado_missao'), 10)
        self.create_subscription(String, f'/{drone}/estado',
                                 self.guardar_estado('estado_drone'), 10)
        self.create_subscription(String, f'/{robo}/estado',
                                 self.guardar_estado('estado_robo'), 10)
        for nome, topico in (('drone', drone), ('robo', robo), ('alvo', alvo_real)):
            self.create_subscription(PoseStamped, f'/{topico}/pose_mundo',
                                     self.guardar_posicao(nome), 10)
        self.create_subscription(DetecaoAlvo, '/alvo/detecao', self.ao_receber_detecao, 10)
        self.create_subscription(EstimativaAlvo, '/alvo/estimativa',
                                 self.ao_receber_estimativa, 10)
        self.create_subscription(
            BatteryState, f'/model/{drone}/battery/linear_battery/state',
            self.ao_receber_bateria, 10)

        # um par de ficheiros por execução, com a data e o modo no nome
        pasta = self.get_parameter('pasta').value
        os.makedirs(pasta, exist_ok=True)
        prefixo = os.path.join(pasta, time.strftime('%Y%m%d_%H%M%S') + '_'
                               + self.get_parameter('modo_intercecao').value)
        self.ficheiro_metricas = open(prefixo + '_metricas.csv', 'w', newline='')
        self.ficheiro_eventos = open(prefixo + '_eventos.csv', 'w', newline='')
        self.csv_metricas = csv.writer(self.ficheiro_metricas)
        self.csv_eventos = csv.writer(self.ficheiro_eventos)
        self.csv_metricas.writerow(COLUNAS)
        self.csv_eventos.writerow(['tempo', 'origem', 'estado'])
        # assim cada resultado fica com os parâmetros que o geraram
        ficheiro_parametros = self.get_parameter('ficheiro_parametros').value
        if ficheiro_parametros and os.path.isfile(ficheiro_parametros):
            shutil.copy(ficheiro_parametros, prefixo + '_parametros.yaml')

        self.create_timer(self.get_parameter('periodo').value, self.escrever_linha)
        self.get_logger().info(f'A gravar em {prefixo}_*.csv')

    def tempo(self):
        """Segundos (simulados) desde que o registo começou."""
        agora = segundos(self.get_clock().now().to_msg())
        if self.instante_inicio is None:
            self.instante_inicio = agora
        return agora - self.instante_inicio

    def guardar_estado(self, chave):
        def callback(msg):
            # só regista eventos quando o estado muda, não a cada mensagem
            if msg.data != self.ultimos[chave]:
                self.ultimos[chave] = msg.data
                self.csv_eventos.writerow([f'{self.tempo():.1f}', chave, msg.data])
                self.ficheiro_eventos.flush()
        return callback

    def guardar_posicao(self, nome):
        def callback(msg):
            posicao = msg.pose.position
            self.posicoes[nome] = (posicao.x, posicao.y, posicao.z)
        return callback

    def ao_receber_detecao(self, msg):
        self.detecao = (msg.x, msg.y, segundos(msg.header.stamp))

    def ao_receber_estimativa(self, msg):
        self.estimativa = (msg.x, msg.y, msg.vx, msg.vy, segundos(msg.header.stamp))

    def ao_receber_bateria(self, msg):
        if msg.capacity > 0.0:
            self.bateria = msg.charge / msg.capacity

    def escrever_linha(self):
        agora = segundos(self.get_clock().now().to_msg())
        drone = self.posicoes.get('drone')
        robo = self.posicoes.get('robo')
        alvo = self.posicoes.get('alvo')

        # deteções e estimativas antigas não interessam para o erro "agora"
        detecao = self.detecao if self.detecao and agora - self.detecao[2] < 1.0 else None
        estimativa = None
        if self.estimativa and agora - self.estimativa[4] < 1.0:
            # leva a estimativa até "agora" para comparar com a pose real deste instante
            x, y, vx, vy, instante = self.estimativa
            decorrido = agora - instante
            estimativa = (x + vx * decorrido, y + vy * decorrido, vx, vy)

        def numero(valor):
            return '' if valor is None or math.isnan(valor) else f'{valor:.2f}'

        def coordenada(ponto, i):
            return numero(ponto[i]) if ponto else ''

        self.csv_metricas.writerow([
            f'{self.tempo():.1f}', self.ultimos['estado_missao'],
            self.ultimos['estado_drone'], self.ultimos['estado_robo'],
            coordenada(drone, 0), coordenada(drone, 1), coordenada(drone, 2),
            coordenada(robo, 0), coordenada(robo, 1),
            coordenada(alvo, 0), coordenada(alvo, 1),
            coordenada(detecao, 0), coordenada(detecao, 1),
            numero(distancia(detecao, alvo)),
            coordenada(estimativa, 0), coordenada(estimativa, 1),
            coordenada(estimativa, 2), coordenada(estimativa, 3),
            numero(distancia(estimativa, alvo)),
            numero(distancia(robo, alvo)),
            numero(distancia(alvo, self.zona_entrega)), numero(self.bateria),
        ])
        self.ficheiro_metricas.flush()

    def destroy_node(self):
        self.ficheiro_metricas.close()
        self.ficheiro_eventos.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = RegistoMetricas()
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
