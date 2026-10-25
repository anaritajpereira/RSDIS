#!/usr/bin/env python3
"""
Converte comandos de velocidade (geometry_msgs/Twist) em força para os dois
motores do barco_1 (propulsão diferencial, como um tanque).

Entrada:  /barco_1/cmd_vel  (por exemplo, do teleop_twist_keyboard)
            linear.x  = velocidade de avanço desejada, m/s
            angular.z = velocidade de rotação desejada, rad/s (positivo = esquerda)
Saída:    força em N para cada motor (std_msgs/Float64), nos tópicos que a ponte
          passa ao plugin Thruster do Gazebo.

Ideia física (controlo em malha aberta, "feedforward"):
  Em regime permanente, a força dos motores iguala o arrasto da água.
  - Avanço: arrasto = (Xu + Xuu*|u|)*u  ->  força total para manter a velocidade u.
  - Rotação: momento de arrasto = Nr*r  ->  os motores estão a +-b do centro, por isso
             uma diferença de força F_dir - F_esq = Nr*r/b cria esse momento.
  Os coeficientes são os do barco_motor/model.sdf (em valor absoluto).
  Nota: a velocidade é relativa à ÁGUA; com corrente, o barco deriva por cima disso.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# Coeficientes de arrasto do WAM-V (iguais aos do model.sdf, sem o sinal)
XU = 51.3      # arrasto linear em avanço, N/(m/s)
XUU = 72.4     # arrasto quadrático em avanço, N/(m/s)^2
NR = 400.0     # arrasto linear em rotação, N.m/(rad/s)
B = 1.348      # distância de cada motor ao centro do barco, m
F_MAX = 718.6  # força máxima de cada motor, N (igual a max_thrust_cmd)


class BarcoTeleop(Node):
    def __init__(self):
        super().__init__('barco_teleop')
        self.declare_parameter('barco', 'barco_1')
        barco = self.get_parameter('barco').value

        base = f'/model/{barco}/joint'
        self.pub_esq = self.create_publisher(
            Float64, f'{base}/left_engine_propeller_joint/cmd_thrust', 10)
        self.pub_dir = self.create_publisher(
            Float64, f'{base}/right_engine_propeller_joint/cmd_thrust', 10)
        self.create_subscription(Twist, f'/{barco}/cmd_vel', self.on_cmd, 10)
        self.get_logger().info(f'À espera de comandos em /{barco}/cmd_vel')

    def on_cmd(self, msg: Twist):
        u = msg.linear.x    # avanço desejado, m/s
        r = msg.angular.z   # rotação desejada, rad/s

        f_total = (XU + XUU * abs(u)) * u      # força total para avançar a u
        df = NR * r / B                         # diferença direita - esquerda para rodar a r

        f_esq = f_total / 2 - df / 2
        f_dir = f_total / 2 + df / 2

        # Limitar à força máxima dos motores
        f_esq = max(-F_MAX, min(F_MAX, f_esq))
        f_dir = max(-F_MAX, min(F_MAX, f_dir))

        self.pub_esq.publish(Float64(data=f_esq))
        self.pub_dir.publish(Float64(data=f_dir))


def main():
    rclpy.init()
    node = BarcoTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ao sair, desligar os motores
        node.pub_esq.publish(Float64(data=0.0))
        node.pub_dir.publish(Float64(data=0.0))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
