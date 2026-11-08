"""
Arranca o oceano, a ponte da bateria e os nós da missão.

Correr a partir da raiz do repositório (o caminho do oceano é relativo a ela):
    ros2 launch missao missao.launch.py
    ros2 launch missao missao.launch.py headless:=true corrente:=false
    ros2 launch missao missao.launch.py modo_intercecao:=perseguir
    ros2 launch missao missao.launch.py parametros:=/caminho/para/outro.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# assume que o launch é corrido na raiz do repo
OCEANO_POR_OMISSAO = os.path.join(
    os.getcwd(), 'oceano_corrente', 'oceano_corrente', 'launch',
    'oceano.launch.py')

# o colcon copia o yaml para o install/, por isso depois de o editar é preciso
# correr o colcon build outra vez (é rápido)
PARAMETROS_POR_OMISSAO = os.path.join(
    get_package_share_directory('missao'), 'config', 'missao.yaml')


def generate_launch_description():
    tempo_simulado = {'use_sim_time': True}

    oceano = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(LaunchConfiguration('oceano_launch')),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'corrente': LaunchConfiguration('corrente'),
        }.items())

    # o MBZIRC não faz a ponte da bateria do drone, por isso fazemos aqui
    ponte_bateria = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/model/drone/battery/linear_battery/state'
                   '@sensor_msgs/msg/BatteryState[ignition.msgs.BatteryState'],
        output='screen')

    modo = LaunchConfiguration('modo_intercecao')
    ficheiro_parametros = LaunchConfiguration('parametros')
    parametros_por_no = {
        # quem arranca a missão é o gestor; o drone fica à espera do comando
        'gestor_missao': {
            'arranque_automatico': ParameterValue(
                LaunchConfiguration('busca_automatica'), value_type=bool),
            'rebocar': ParameterValue(LaunchConfiguration('reboque'), value_type=bool)},
        'robo_maritimo_missao': {'modo_intercecao': modo},
        'registo_metricas': {'modo_intercecao': modo,
                             'pasta': LaunchConfiguration('pasta_metricas'),
                             'ficheiro_parametros': ficheiro_parametros},
    }
    # ordem: primeiro o yaml, depois o que vem dos argumentos do launch (ganha o último)
    nos_missao = [
        Node(package='missao', executable=executavel, output='screen',
             parameters=[tempo_simulado, ficheiro_parametros,
                         parametros_por_no.get(executavel, {})])
        for executavel in ('poses', 'detetor', 'estimativa_deriva', 'drone_missao',
                           'robo_maritimo_missao', 'gestor_missao', 'registo_metricas')]

    return LaunchDescription([
        DeclareLaunchArgument(
            'oceano_launch', default_value=OCEANO_POR_OMISSAO,
            description='Caminho do oceano.launch.py'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true = só o servidor, sem janela do Gazebo'),
        DeclareLaunchArgument(
            'corrente', default_value='true',
            description='Arrancar o nó da corrente'),
        DeclareLaunchArgument(
            'busca_automatica', default_value='true',
            description='false = a missão só arranca com '
                        'ros2 service call /missao/iniciar std_srvs/srv/Trigger'),
        DeclareLaunchArgument(
            'modo_intercecao', default_value='intercetar',
            choices=['perseguir', 'intercetar'],
            description='Barco robô aponta à posição atual ou ao ponto de interceção'),
        DeclareLaunchArgument(
            'reboque', default_value='true',
            description='false = o robô marítimo fica a acompanhar o alvo, sem o rebocar'),
        DeclareLaunchArgument(
            'pasta_metricas', default_value=os.path.join(os.getcwd(), 'metricas'),
            description='Onde o registo_metricas grava os CSV'),
        DeclareLaunchArgument(
            'parametros', default_value=PARAMETROS_POR_OMISSAO,
            description='Ficheiro yaml com os parâmetros dos nós'),
        oceano,
        ponte_bateria,
        *nos_missao,
    ])
