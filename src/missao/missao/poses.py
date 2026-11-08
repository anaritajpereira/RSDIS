"""Separa as poses do Gazebo numa PoseStamped por entidade."""
import ctypes
import json
import signal
import subprocess
import threading

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node

PR_SET_PDEATHSIG = 1


def morrer_com_o_pai():
    """Pede ao kernel que termine o processo filho quando o pai morre."""
    # sem isto, se o nó morrer o ign topic fica a correr sozinho
    ctypes.CDLL('libc.so.6').prctl(PR_SET_PDEATHSIG, signal.SIGTERM)


class Poses(Node):
    """Publica /<nome>/pose_mundo a partir do dynamic_pose/info do Gazebo."""

    def __init__(self):
        super().__init__('poses')
        self.declare_parameter('topico_gazebo',
                               '/world/oceano_corrente/dynamic_pose/info')
        self.declare_parameter('frame_mundo', 'oceano_corrente')
        # nomes no Gazebo (não os podemos mudar) e os nomes que usamos nos
        # tópicos, pela mesma ordem: /robo_maritimo/pose_mundo, /barco/pose_mundo...
        self.declare_parameter(
            'entidades', ['drone', 'barco_1', 'barco_2', 'marca_vermelha'])
        self.declare_parameter(
            'nomes', ['drone', 'robo_maritimo', 'barco', 'marca_vermelha'])

        self.frame_mundo = self.get_parameter('frame_mundo').value
        entidades = self.get_parameter('entidades').value
        nomes = self.get_parameter('nomes').value
        # chave: nome no Gazebo; valor: (nosso nome, publicador)
        self.publicadores = {
            entidade: (nome, self.create_publisher(
                PoseStamped, f'/{nome}/pose_mundo', 10))
            for entidade, nome in zip(entidades, nomes)}
        self.entidades_encontradas = set()

        # tentámos com a ros_ign_bridge, mas no Galactic o Pose_V chega como
        # TFMessage com o child_frame_id vazio e não se sabe qual pose é de quem.
        # Lendo o "ign topic" em JSON os nomes vêm lá.
        topico = self.get_parameter('topico_gazebo').value
        self.processo_ign = subprocess.Popen(
            ['ign', 'topic', '-e', '--json-output', '-t', topico],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            universal_newlines=True, preexec_fn=morrer_com_o_pai)
        # thread à parte porque ler o stdout bloqueia
        threading.Thread(target=self.ler_gazebo, daemon=True).start()
        self.get_logger().info(f'À espera de poses em {topico} (Gazebo)')

    def ler_gazebo(self):
        for linha in self.processo_ign.stdout:
            try:
                msg_gazebo = json.loads(linha)
            except ValueError:
                continue
            self.publicar_poses(msg_gazebo)

    def publicar_poses(self, msg_gazebo):
        instante = msg_gazebo.get('header', {}).get('stamp', {})
        for pose_gazebo in msg_gazebo.get('pose', []):
            entidade = pose_gazebo.get('name')
            if entidade not in self.publicadores:
                continue
            nome, publicador = self.publicadores[entidade]
            if entidade not in self.entidades_encontradas:
                self.entidades_encontradas.add(entidade)
                no_gazebo = f' ({entidade} no Gazebo)' if nome != entidade else ''
                self.get_logger().info(f'Pose de {nome}{no_gazebo} encontrada')

            pose_ros = PoseStamped()
            # o JSON do protobuf não manda campos que sejam 0 e manda os int64 como
            # string, daí os .get(..., 0) e os int()
            pose_ros.header.stamp.sec = int(instante.get('sec', 0))
            pose_ros.header.stamp.nanosec = int(instante.get('nsec', 0))
            pose_ros.header.frame_id = self.frame_mundo
            posicao = pose_gazebo.get('position', {})
            pose_ros.pose.position.x = float(posicao.get('x', 0.0))
            pose_ros.pose.position.y = float(posicao.get('y', 0.0))
            pose_ros.pose.position.z = float(posicao.get('z', 0.0))
            orientacao = pose_gazebo.get('orientation', {})
            pose_ros.pose.orientation.x = float(orientacao.get('x', 0.0))
            pose_ros.pose.orientation.y = float(orientacao.get('y', 0.0))
            pose_ros.pose.orientation.z = float(orientacao.get('z', 0.0))
            pose_ros.pose.orientation.w = float(orientacao.get('w', 0.0))
            publicador.publish(pose_ros)

    def destroy_node(self):
        self.processo_ign.terminate()
        super().destroy_node()


def main():
    rclpy.init()
    node = Poses()
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
