"""
Lança o oceano com corrente: mundo + drone + barco com motores + pontes + nós Python.

Uso (a partir da pasta oceano_corrente, com o workspace do MBZIRC já "sourced"):
    ros2 launch launch/oceano.launch.py
    ros2 launch launch/oceano.launch.py corrente:=false     # sem o nó Python
    ros2 launch launch/oceano.launch.py headless:=true      # sem janela do Gazebo (mais leve)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Pasta deste projeto (um nível acima de launch/)
AQUI = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    # 1) O Gazebo procura mundos e modelos pelo nome nas pastas desta variável.
    #    Acrescentamos worlds/ (o nosso mundo) e models/ (o barco_motor)
    #    à frente das pastas do MBZIRC.
    resource_path = os.pathsep.join([
        os.path.join(AQUI, 'worlds'),
        os.path.join(AQUI, 'models'),
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
    ])

    # 2) O launch do próprio MBZIRC arranca o Gazebo com o nosso mundo e faz o
    #    spawn do drone a partir do YAML (com controlador e pontes ROS incluídos).
    competition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mbzirc_ign'), 'launch',
            'competition.launch.py')),
        launch_arguments={
            'world': 'oceano_corrente',
            'config_file': os.path.join(AQUI, 'config', 'drone.yaml'),
            'headless': LaunchConfiguration('headless'),
        }.items())

    # 3) Pontes ROS -> Gazebo (o "]" indica sentido único):
    #    - a corrente;
    #    - a força de cada motor do barco_1 (mesmo nome de tópico nos dois lados).
    motor = '/model/barco_1/joint/{}_engine_propeller_joint/cmd_thrust'
    ponte = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/ocean_current@geometry_msgs/msg/Vector3]ignition.msgs.Vector3d',
            motor.format('left') + '@std_msgs/msg/Float64]ignition.msgs.Double',
            motor.format('right') + '@std_msgs/msg/Float64]ignition.msgs.Double',
        ],
        output='screen')

    # 4) O nosso nó Python que decide a corrente.
    no_corrente = ExecuteProcess(
        cmd=['python3', os.path.join(AQUI, 'scripts', 'corrente.py')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('corrente')))

    # 5) Conversor teleop -> motores do barco_1 (fica à espera em /barco_1/cmd_vel).
    no_barco = ExecuteProcess(
        cmd=['python3', os.path.join(AQUI, 'scripts', 'barco_teleop.py')],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('corrente', default_value='true',
                              description='Arrancar o nó Python da corrente'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true = só o servidor, sem janela do Gazebo'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),
        competition,
        ponte,
        no_corrente,
        no_barco,
    ])
