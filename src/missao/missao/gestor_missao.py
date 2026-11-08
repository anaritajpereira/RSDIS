"""Gestor da missão: arranca a busca, pede o resgate e o reboque, e faz o hand-off."""
from missao_interfaces.action import Rebocar, Resgatar
from missao_interfaces.msg import EstimativaAlvo
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

ESPERAR = 'ESPERAR'  # à espera que os nós estejam prontos / do pedido de início
BUSCA = 'BUSCA'
RESGATE = 'RESGATE'
HAND_OFF = 'HAND_OFF'
REBOQUE = 'REBOQUE'
CONCLUIDA = 'CONCLUIDA'
FALHOU = 'FALHOU'


class GestorMissao(Node):
    """Coordena o drone e o robô marítimo e publica o estado em /missao/estado."""

    def __init__(self):
        super().__init__('gestor_missao')
        parametros = {
            'drone': 'drone',
            'robo_maritimo': 'robo_maritimo',
            # a false fica à espera de: ros2 service call /missao/iniciar std_srvs/srv/Trigger
            'arranque_automatico': True,
            'tentativas_resgate': 3,
            # fase 4: depois do resgate o robô reboca o alvo até à zona de entrega
            # (por omissão, o sítio de onde o robô marítimo sai)
            'rebocar': True,
            'zona_entrega_x': 15.0,
            'zona_entrega_y': 10.0,
            'frequencia': 2.0,
        }
        for nome, valor in parametros.items():
            self.declare_parameter(nome, valor)

        drone, robo = self.param('drone'), self.param('robo_maritimo')
        self.pub_estado = self.create_publisher(String, '/missao/estado', 10)
        self.pub_comando_drone = self.create_publisher(
            String, f'/{drone}/comando', 10)
        self.create_subscription(
            String, f'/{drone}/estado', self.ao_receber_estado_drone, 10)
        self.create_subscription(
            String, f'/{robo}/estado', self.ao_receber_estado_robo, 10)
        self.create_subscription(
            EstimativaAlvo, '/alvo/estimativa', self.ao_receber_estimativa, 10)
        self.cliente_resgate = ActionClient(self, Resgatar, f'/{robo}/resgatar')
        self.cliente_reboque = ActionClient(self, Rebocar, f'/{robo}/rebocar')
        self.create_service(Trigger, '/missao/iniciar', self.ao_pedir_inicio)

        self.estado = ESPERAR
        self.estado_drone = None
        self.estado_robo = None
        self.estimativa = None
        self.inicio_pedido = self.param('arranque_automatico')
        self.pedido_resgate = None
        self.tentativas = 0
        self.drone_ja_voou = False
        # None = ainda não acabou; True/False = acabou bem/mal
        self.reboque_correu_bem = None
        self.create_timer(1.0 / self.param('frequencia'), self.ciclo)
        self.get_logger().info('Gestor pronto')

    def param(self, nome):
        return self.get_parameter(nome).value

    def mudar_estado(self, novo_estado, motivo=''):
        texto = f'{self.estado} -> {novo_estado}'
        self.get_logger().info(f'{texto}: {motivo}' if motivo else texto)
        self.estado = novo_estado
        self.pub_estado.publish(String(data=self.estado))

    def ao_pedir_inicio(self, pedido, resposta):
        if self.estado != ESPERAR:
            resposta.success = False
            resposta.message = f'A missão já está em {self.estado}'
        else:
            self.inicio_pedido = True
            resposta.success = True
            resposta.message = 'Missão vai arrancar'
        return resposta

    def ao_receber_estado_drone(self, msg):
        self.estado_drone = msg.data
        if msg.data not in ('POUSADO', 'DESCOLAR'):
            self.drone_ja_voou = True

    def ao_receber_estado_robo(self, msg):
        self.estado_robo = msg.data

    def ao_receber_estimativa(self, msg):
        self.estimativa = msg

    def comandar_drone(self, comando):
        self.pub_comando_drone.publish(String(data=comando))

    def ciclo(self):
        self.pub_estado.publish(String(data=self.estado))

        if self.estado == ESPERAR:
            prontos = (self.estado_drone == 'POUSADO' and self.estado_robo is not None
                       and self.cliente_resgate.server_is_ready())
            if not prontos:
                self.get_logger().info('À espera do drone e do robô marítimo...',
                                       throttle_duration_sec=10.0)
            elif self.inicio_pedido:
                self.mudar_estado(BUSCA, 'drone vai procurar o alvo')

        elif self.estado == BUSCA:
            # reenvia até o drone sair de POUSADO (a 1ª mensagem pode perder-se)
            if self.estado_drone == 'POUSADO' and not self.drone_ja_voou:
                self.comandar_drone('buscar')
            elif self.estado_drone == 'POUSADO':
                self.mudar_estado(FALHOU, 'o drone voltou sem encontrar o alvo')
            elif self.estimativa is not None:
                self.pedir_resgate()

        elif self.estado == RESGATE:
            if self.estado_drone == 'POUSADO':
                # acabou a bateria antes do robô chegar; o robô continua sozinho
                self.get_logger().warning(
                    'Drone já em terra; o robô continua com a última deriva',
                    throttle_duration_sec=30.0)

        elif self.estado == HAND_OFF:
            if self.estado_drone not in ('REGRESSAR', 'ATERRAR', 'POUSADO'):
                self.comandar_drone('regressar')
            elif self.estado_drone == 'POUSADO':
                if not self.param('rebocar'):
                    self.mudar_estado(CONCLUIDA,
                                      'drone em terra e robô a acompanhar o alvo')
                else:
                    # o reboque já começou no hand-off; aqui só se espera por ele
                    self.mudar_estado(REBOQUE, 'drone em terra, robô a rebocar o alvo')

        elif self.estado == REBOQUE:
            if self.reboque_correu_bem is True:
                self.mudar_estado(CONCLUIDA, 'alvo entregue e drone em terra')
            elif self.reboque_correu_bem is False:
                self.mudar_estado(FALHOU, 'o reboque não correu bem')

    def pedir_resgate(self):
        self.tentativas += 1
        pedido = Resgatar.Goal()
        pedido.x, pedido.y = self.estimativa.x, self.estimativa.y
        self.mudar_estado(
            RESGATE, f'alvo em ({pedido.x:.1f}, {pedido.y:.1f}), '
            f'tentativa {self.tentativas}')
        envio = self.cliente_resgate.send_goal_async(
            pedido, feedback_callback=self.ao_receber_feedback)
        envio.add_done_callback(self.ao_aceitar_pedido)

    def ao_aceitar_pedido(self, futuro):
        self.pedido_resgate = futuro.result()
        if not self.pedido_resgate.accepted:
            self.resgate_falhou('o robô recusou o pedido')
            return
        self.pedido_resgate.get_result_async().add_done_callback(self.ao_terminar_resgate)

    def ao_receber_feedback(self, msg):
        feedback = msg.feedback
        self.get_logger().info(
            f'Robô em {feedback.estado}, distância {feedback.distancia:.0f} m',
            throttle_duration_sec=5.0)

    def ao_terminar_resgate(self, futuro):
        resultado = futuro.result().result
        if resultado.sucesso:
            self.get_logger().info(f'Robô confirmou: {resultado.mensagem}')
            self.mudar_estado(HAND_OFF, 'drone pode regressar')
            self.comandar_drone('regressar')
            if self.param('rebocar'):
                self.pedir_reboque()
        else:
            self.resgate_falhou(resultado.mensagem)

    def pedir_reboque(self):
        pedido = Rebocar.Goal()
        pedido.x, pedido.y = self.param('zona_entrega_x'), self.param('zona_entrega_y')
        self.get_logger().info(f'A pedir o reboque para ({pedido.x:.1f}, {pedido.y:.1f})')
        envio = self.cliente_reboque.send_goal_async(
            pedido, feedback_callback=self.ao_receber_feedback)
        envio.add_done_callback(self.ao_aceitar_reboque)

    def ao_aceitar_reboque(self, futuro):
        pedido = futuro.result()
        if not pedido.accepted:
            self.get_logger().error('O robô recusou o pedido de reboque')
            self.reboque_correu_bem = False
            return
        pedido.get_result_async().add_done_callback(self.ao_terminar_reboque)

    def ao_terminar_reboque(self, futuro):
        resultado = futuro.result().result
        if resultado.sucesso:
            self.get_logger().info(f'Reboque concluído: {resultado.mensagem}')
        else:
            self.get_logger().error(f'Reboque falhou: {resultado.mensagem}')
        self.reboque_correu_bem = resultado.sucesso

    def resgate_falhou(self, motivo):
        if self.tentativas < self.param('tentativas_resgate'):
            self.get_logger().warning(f'Resgate falhou ({motivo}); a tentar outra vez')
            # volta a BUSCA e no próximo ciclo pede outra vez com a estimativa atual
            self.mudar_estado(BUSCA)
        else:
            self.mudar_estado(FALHOU, f'resgate falhou {self.tentativas} vezes: {motivo}')

    def destroy_node(self):
        # sem isto o ActionClient queixa-se ao ser apagado depois do nó
        self.cliente_resgate.destroy()
        self.cliente_reboque.destroy()
        super().destroy_node()


def main():
    rclpy.init()
    node = GestorMissao()
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
