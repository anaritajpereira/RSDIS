"""Deteta o painel vermelho na câmara do drone e projeta-o no mundo."""
from collections import deque

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from missao.geometria import (matriz_de_quaternion, matriz_de_rpy,
                              pixel_para_mundo)
from missao_interfaces.msg import DetecaoAlvo
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def segundos(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Detetor(Node):
    """Segmenta o vermelho em HSV e publica a posição do alvo no mundo."""

    def __init__(self):
        super().__init__('detetor')
        parametros = {
            'drone': 'drone',
            'slot': 'slot0',
            # no OpenCV a matiz vai de 0 a 180 e o vermelho está nas duas pontas,
            # por isso são precisas duas máscaras
            'matiz_baixa_max': 10,
            'matiz_alta_min': 170,
            'saturacao_min': 100,
            'valor_min': 50,
            'nucleo_morfologia': 5,
            'area_min': 30.0,
            'area_confianca_total': 400.0,  # a partir desta área (px) a confiança é 1
            # posição da câmara slot0 em relação ao drone (tirada do modelo do MBZIRC)
            'camera_xyz': [0.1, 0.0, 0.053],
            'camera_rpy': [0.0, 0.0, 0.0],
            # altura do plano onde se projeta. O painel está a ~1.2 m da água;
            # com 0 o erro era de 5 a 12 m e variava com a distância, o que
            # estragava a estimativa da deriva (ver README)
            'altura_plano': 1.2,
            'distancia_max': 300.0,  # perto do horizonte a projeção já não presta
            'atraso_pose_max': 0.1,
            'frame_mundo': 'oceano_corrente',
            'publicar_depuracao': False,
        }
        for nome, valor in parametros.items():
            self.declare_parameter(nome, valor)

        self.conversor_imagem = CvBridge()
        self.intrinsecos = None
        # a imagem chega com algum atraso, por isso guardamos as últimas poses e
        # usamos a que tem o stamp mais perto do da imagem
        self.poses_recentes = deque(maxlen=100)

        drone, slot = self.param('drone'), self.param('slot')
        self.create_subscription(
            CameraInfo, f'/{drone}/{slot}/camera_info', self.ao_receber_info, 10)
        self.create_subscription(
            PoseStamped, f'/{drone}/pose_mundo', self.ao_receber_pose, 50)
        self.create_subscription(
            Image, f'/{drone}/{slot}/image_raw', self.ao_receber_imagem, 2)
        self.pub_detecao = self.create_publisher(DetecaoAlvo, '/alvo/detecao', 10)
        self.pub_depuracao = self.create_publisher(
            Image, '/alvo/imagem_depuracao', 2)
        self.get_logger().info(f'À espera de imagens em /{drone}/{slot}/image_raw')

    def param(self, nome):
        return self.get_parameter(nome).value

    def ao_receber_info(self, msg):
        if self.intrinsecos is None:
            self.get_logger().info(
                f'Intrínsecos recebidos: fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} '
                f'cx={msg.k[2]:.1f} cy={msg.k[5]:.1f}')
        self.intrinsecos = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def ao_receber_pose(self, msg):
        self.poses_recentes.append((segundos(msg.header.stamp), msg.pose))

    def pose_mais_proxima(self, instante):
        if not self.poses_recentes:
            return None
        instante_pose, pose = min(
            self.poses_recentes,
            key=lambda instante_e_pose: abs(instante_e_pose[0] - instante))
        if abs(instante_pose - instante) > self.param('atraso_pose_max'):
            return None
        return pose

    def ao_receber_imagem(self, msg):
        if self.intrinsecos is None:
            return
        imagem = self.conversor_imagem.imgmsg_to_cv2(msg, 'bgr8')
        mascara = self.segmentar(imagem)
        contornos, _ = cv2.findContours(
            mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # fica só com o maior blob, o resto normalmente é ruído
        maior_contorno = max(contornos, key=cv2.contourArea, default=None)
        area = cv2.contourArea(maior_contorno) if maior_contorno is not None else 0.0

        if self.param('publicar_depuracao'):
            self.publicar_depuracao(imagem, mascara, maior_contorno, msg.header)
        if area < self.param('area_min'):
            return

        # centro do blob a partir dos momentos
        momentos = cv2.moments(maior_contorno)
        centro_u = momentos['m10'] / momentos['m00']
        centro_v = momentos['m01'] / momentos['m00']
        pose = self.pose_mais_proxima(segundos(msg.header.stamp))
        if pose is None:
            self.get_logger().warning('Alvo visto mas sem pose do drone nesse instante',
                                      throttle_duration_sec=5.0)
            return

        orientacao = pose.orientation
        posicao_drone = (pose.position.x, pose.position.y, pose.position.z)
        ponto_mundo = pixel_para_mundo(
            centro_u, centro_v, self.intrinsecos, posicao_drone,
            matriz_de_quaternion(orientacao.x, orientacao.y, orientacao.z,
                                 orientacao.w),
            self.param('camera_xyz'), matriz_de_rpy(*self.param('camera_rpy')),
            self.param('altura_plano'))
        if ponto_mundo is None:
            return
        distancia = float(np.hypot(ponto_mundo[0] - posicao_drone[0],
                                   ponto_mundo[1] - posicao_drone[1]))
        if distancia > self.param('distancia_max'):
            return

        detecao = DetecaoAlvo()
        detecao.header.stamp = msg.header.stamp
        detecao.header.frame_id = self.param('frame_mundo')
        detecao.x, detecao.y = float(ponto_mundo[0]), float(ponto_mundo[1])
        # confiança simples: quanto maior o painel na imagem, mais confiamos
        detecao.confianca = min(1.0, area / self.param('area_confianca_total'))
        self.pub_detecao.publish(detecao)
        self.get_logger().info(
            f'Alvo em ({detecao.x:.1f}, {detecao.y:.1f}), a {distancia:.0f} m; '
            f'área {area:.0f} px', throttle_duration_sec=2.0)

    def segmentar(self, imagem):
        hsv = cv2.cvtColor(imagem, cv2.COLOR_BGR2HSV)
        saturacao_min = self.param('saturacao_min')
        valor_min = self.param('valor_min')
        mascara_matiz_baixa = cv2.inRange(
            hsv, (0, saturacao_min, valor_min),
            (self.param('matiz_baixa_max'), 255, 255))
        mascara_matiz_alta = cv2.inRange(
            hsv, (self.param('matiz_alta_min'), saturacao_min, valor_min),
            (180, 255, 255))
        mascara = cv2.bitwise_or(mascara_matiz_baixa, mascara_matiz_alta)
        tamanho = self.param('nucleo_morfologia')
        nucleo = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (tamanho, tamanho))
        # open tira pontinhos soltos, close tapa buracos dentro do painel
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, nucleo)
        return cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, nucleo)

    def publicar_depuracao(self, imagem, mascara, contorno, header):
        imagem_anotada = imagem.copy()
        imagem_anotada[mascara > 0] = (0, 255, 0)
        if contorno is not None:
            cv2.drawContours(imagem_anotada, [contorno], -1, (255, 255, 0), 2)
        msg = self.conversor_imagem.cv2_to_imgmsg(imagem_anotada, 'bgr8')
        msg.header = header
        self.pub_depuracao.publish(msg)


def main():
    rclpy.init()
    node = Detetor()
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
