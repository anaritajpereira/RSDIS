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



#------------------------------------------------------------------------


from math import atan2, cos, pi, sqrt
import rclpy

from rclpy.node import Node

from rclpy.qos import DurabilityPolicy, QoSProfile
# Para configurar a QoS (Quality of Service) do tópico /posicao.

from geometry_msgs.msg import PointStamped   # ponto (x, y, z) + cabeçalho (referencial e tempo)
from nav_msgs.msg import Odometry            # posição, orientação e velocidade de um robô
from std_msgs.msg import Bool, Float64  


def euclidean_distance(x, y, gx, gy):
    """Distância em linha reta do barco (x, y) ao destino (gx, gy)."""
    return sqrt((gx - x) ** 2 + (gy - y) ** 2)


def steering_angle(x, y, gx, gy):
    """Ângulo, no referencial do mundo, da linha que vai do barco ao destino."""
    return atan2(gy - y, gx - x)


def normalizar_angulo(a):
    """Põe um ângulo no intervalo [-pi, pi] para que o barco não ande Às voltas e faça um trajecto mais pequeno""" # alteração ao tuturial do ros go to
    while a > pi:          # se passou de +180°...
        a -= 2 * pi        # ...subtrai uma volta (360°)
    while a < -pi:         # se passou de -180°...
        a += 2 * pi        # ...soma uma volta
    return a


def linear_vel(distancia, erro_rumo, ganho, vel_max):
    """Avanço desejado (m/s)."""
    v = min(ganho * distancia, vel_max)
    # Controlo proporcional (como no tutorial): quanto mais longe, mais depressa.
    # O min() limita a vel_max (correção: sem limite, a 200 m pedia 60 m/s).
    return v * max(0.0, cos(erro_rumo))
    # Correção: multiplica pelo cosseno do erro de rumo.
    #   erro = 0°        → cos = 1    → avanço total (aponta ao destino)
    #   erro = 60°       → cos = 0.5  → metade do avanço
    #   erro ≥ 90°       → cos ≤ 0    → max(0, ...) = 0 → não avança, só roda
    # Assim o barco roda primeiro no sítio e não faz curvas enormes.


def angular_vel(erro_rumo, ganho, rot_max):
    """Rotação desejada (rad/s)."""
    return max(-rot_max, min(rot_max, ganho * erro_rumo))
    # Sinal: erro positivo (destino à esquerda) → rotação positiva (roda à esquerda).




class BarcoMissao(Node):

    def __init__(self):
        super().__init__('barco_missao') #inicializar o nó 

        # ---------- Parâmetros ----------
        # Declaro os parâmtros para os poder alterar sem mudar o código 
        self.declare_parameter('barco', 'barco_1')    # nome do modelo no Gazebo
        self.declare_parameter('tolerancia', 2.0)     # m: abaixo disto, "cheguei"
        self.declare_parameter('ganho_lin', 0.3)      # (m/s) por metro de distância
        self.declare_parameter('vel_max', 2.0)        # m/s (relativa à água)
        self.declare_parameter('ganho_ang', 1.0)      # (rad/s) por radiano de erro
        self.declare_parameter('rot_max', 0.5)        # rad/s
        barco = self.get_parameter('barco').value     # lê o valor barco_1 

        qos_posicao = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL) # define as regras para a apresetação dos dados.
        # depth=1 dá apenas o ultimo valor lido
        #durability: Transcient_local dá ol ultimos valores obtidos antes de o topico ser publicado

        #tópicos que vai subscrever 

        self.create_subscription(PointStamped, '/posicao', self.on_posicao, qos_posicao) # subscreve se ao tópico para obter a posição

        self.create_subscription(Odometry, f'/model/{barco}/odometry', self.on_odom, 10) # dá a propria posição, velocidade e para onde aponta o barco

        #tópicos que vai publicar
         
        motor = f'/model/{barco}/joint/{{}}_engine_propeller_joint/cmd_thrust'
        self.pub_esq = self.create_publisher(Float64, motor.format('left'), 10)   # vira o motor para a esquerda
        self.pub_dir = self.create_publisher(Float64, motor.format('right'), 10)  # vira o motor para a direira 
        self.pub_cheguei = self.create_publisher(Bool, '/cheguei', 10)            # publica a dizer que chegou ao seu destino 


        # ---------- Estado interno ----------
        self.pose = None        # (x, y, rumo) do barco; None até chegar a 1.ª odometria
        self.destino = None     # (x, y) do destino; None até o drone enviar algum
        self.a_navegar = False  # True enquanto está a caminho de um destino

        self.create_timer(0.1, self.ciclo_controlo)

    def on_odom(self, msg):
        """Chamado a cada mensagem de odometria dá a posição """
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_de_quaternion(p.orientation))  

    def on_posicao(self, msg):
        """Chamado quando o drone publica um destino."""
        if msg.header.frame_id not in ('', 'world'):
            # Proteção: os números só fazem sentido se estiverem no referencial do
            # mundo, que é o mesmo da odometria. Se vierem noutro, ignora.
            self.get_logger().warn(
                f'Destino no referencial "{msg.header.frame_id}"; esperava "world". Ignorado.')
            return

    
        novo = (msg.point.x, msg.point.y)       # z ignorado: estamos à superfície
        tol = self.get_parameter('tolerancia').value #pega no tolerancia definida anteriormente


        if self.pose and euclidean_distance(self.pose[0], self.pose[1], *novo) <= tol:
            return

        if not self.a_navegar:
            self.get_logger().info(f'Novo destino: x={novo[0]:.1f}, y={novo[1]:.1f}')  '''perceber esta linha'''
        self.destino = novo         # atualiza o destino 
        self.a_navegar = True       # volta a ativar o esta

     def ciclo_controlo(self):
        if not self.a_navegar or self.pose is None: # se não está a navegar então não entra nesta função
            return

        x, y, rumo = self.pose      # dá me as posições do barco separadas
        gx, gy = self.destino       # posições de destino tmb separadas
        tol = self.get_parameter('tolerancia').value  # vai-me buscar a tolerancia que defini anteriormente

        distancia = euclidean_distance(x, y, gx, gy) # diz me a distância em linha reta ao alvo
        if distancia <= tol:
            # ----- Chegou -----
            self.parar_motores()                         # motores a zero
            self.a_navegar = False                       # deixa de navegar
            self.pub_cheguei.publish(Bool(data=True))    # avisa o drone
            self.get_logger().info(f'Cheguei (a {distancia:.1f} m do destino).')
            return

        # ----- Ainda a caminho -----
        erro_rumo = normalizar_angulo(steering_angle(x, y, gx, gy) - rumo) #dá me o ângulo para onde deveria apontar. ângulo até ao destino menos o ângulo para onde aponta

        u = linear_vel(distancia, erro_rumo,
                       self.get_parameter('ganho_lin').value,
                       self.get_parameter('vel_max').value)
        # Avanço desejado. Os parâmetros são lidos em cada ciclo, por isso podes
        # afiná-los com o nó a correr:  ros2 param set /barco_missao ganho_lin 0.5

        r = angular_vel(erro_rumo,
                        self.get_parameter('ganho_ang').value,
                        self.get_parameter('rot_max').value)
        # Rotação desejada.

        f_esq, f_dir = forca_motores(u, r)               # converte em forças
        self.pub_esq.publish(Float64(data=f_esq))        # envia ao motor esquerdo
        self.pub_dir.publish(Float64(data=f_dir))        # envia ao motor direito
        # Os motores mantêm a última força recebida, por isso o ciclo tem de
        # continuar a publicar enquanto navega.

    def parar_motores(self):
        self.pub_esq.publish(Float64(data=0.0))
        self.pub_dir.publish(Float64(data=0.0))

    def main(args=None):
    rclpy.init(args=args)       # inicializa o ROS 2 neste processo
    node = BarcoMissao()        # cria o nó (corre o __init__)
    try:
        rclpy.spin(node)
        # "spin" = fica em ciclo à espera de mensagens e timers e chama os
        # callbacks. Só sai com Ctrl+C.
    except KeyboardInterrupt:
        pass                    # Ctrl+C: sai sem mostrar o erro
    finally:
        # Corre sempre ao sair (com Ctrl+C ou com erro):
        node.parar_motores()    # segurança: não deixar o barco com os motores ligados
        node.destroy_node()     # liberta os recursos do nó
        rclpy.shutdown()        # fecha o ROS 2 neste processo


if __name__ == '__main__':
    main()






    
