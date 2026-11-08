#!/usr/bin/env python3
"""
Nó ROS 2 que define a corrente marítima do mundo oceano_corrente.

Publica um geometry_msgs/Vector3 (m/s, referencial do mundo) em /ocean_current.
A ponte ros_ign_bridge passa-o para o Gazebo, onde o plugin Hydrodynamics de
cada barco o usa para calcular o arrasto com a velocidade RELATIVA à água.

Parâmetros (podem ser mudados com --ros-args -p nome:=valor):
  velocidade      módulo da corrente em m/s              (default 0.5)
  direcao_graus   direção inicial, 0 = +x, 90 = +y       (default 0.0)
  rotacao_graus_s quanto a direção roda por segundo      (default 0.0 -> constante)
  rajada          amplitude de variação do módulo, m/s   (default 0.0 -> sem rajadas)
  periodo_rajada  período da variação do módulo, s       (default 30.0)
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3


class Corrente(Node):
    def __init__(self):
        super().__init__('corrente')

        self.declare_parameter('velocidade', 0.5)
        self.declare_parameter('direcao_graus', 0.0)
        self.declare_parameter('rotacao_graus_s', 0.0)
        self.declare_parameter('rajada', 0.0)
        self.declare_parameter('periodo_rajada', 30.0)

        self.pub = self.create_publisher(Vector3, '/ocean_current', 10)
        self.dt = 0.1          # publica a 10 Hz; a corrente muda devagar, chega bem
        self.t = 0.0
        self.create_timer(self.dt, self.publicar)

    def publicar(self):
        p = lambda nome: self.get_parameter(nome).value

        # direção: valor inicial + rotação acumulada ao longo do tempo
        ang = math.radians(p('direcao_graus') + p('rotacao_graus_s') * self.t)

        # módulo: base + oscilação sinusoidal (simula variação de maré/rajadas)
        mod = p('velocidade')
        if p('rajada') > 0.0:
            mod += p('rajada') * math.sin(2 * math.pi * self.t / p('periodo_rajada'))

        msg = Vector3(x=mod * math.cos(ang), y=mod * math.sin(ang), z=0.0)
        self.pub.publish(msg)
        self.t += self.dt


def main():
    rclpy.init()
    node = Corrente()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
