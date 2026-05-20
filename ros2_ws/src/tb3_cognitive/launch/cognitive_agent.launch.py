"""
cognitive_agent.launch.py — Lanza toda la capa cognitiva
---------------------------------------------------------

Lanza en orden:
  1. nlp_intent_node    — clasifica texto → /cognitive/command
  2. cognitive_agent_node — ejecuta comandos → NAV2 / cmd_vel

Prerequisito:
  NAV2 corriendo:
    ros2 launch tb3_bringup navigation.launch.py

Uso completo:
  # Terminal 1 — stack de navegación
  ros2 launch tb3_bringup navigation.launch.py

  # Terminal 2 — capa cognitiva
  ros2 launch tb3_cognitive cognitive_agent.launch.py

  # Terminal 3 — enviar comando
  ros2 topic pub --once /nlp/input std_msgs/String "data: 've a la cocina'"

  # Observar
  ros2 topic echo /cognitive/command
  ros2 topic echo /cognitive/status

Verificación:
  ros2 node list | grep -E "nlp|agent"
  ros2 topic list | grep cognitive
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_AI_VENV_SITE   = '/opt/ai-venv/lib/python3.12/site-packages'
_SYSTEM_PYTHONPATH = os.environ.get('PYTHONPATH', '')
_pythonpath = ':'.join(filter(None, [_AI_VENV_SITE, _SYSTEM_PYTHONPATH]))


def generate_launch_description():

    pkg = get_package_share_directory('tb3_cognitive')
    default_waypoints = os.path.join(pkg, 'config', 'waypoints.yaml')

    declare_waypoints = DeclareLaunchArgument(
        'waypoints_file',
        default_value=default_waypoints,
        description='Ruta al YAML con coordenadas de waypoints')

    declare_model = DeclareLaunchArgument(
        'model',
        default_value='sentence-transformers/paraphrase-multilingual-MiniLM-L12-v2',
        description='Modelo sentence-transformers para NLP')

    nlp_node = Node(
        package='tb3_cognitive',
        executable='nlp_intent_node',
        name='nlp_intent_node',
        output='screen',
        prefix='/opt/ai-venv/bin/python3',
        additional_env={'PYTHONPATH': _pythonpath},
        parameters=[{
            'model':  LaunchConfiguration('model'),
            'device': -1,
        }])

    agent_node = Node(
        package='tb3_cognitive',
        executable='cognitive_agent_node',
        name='cognitive_agent_node',
        output='screen',
        prefix='/opt/ai-venv/bin/python3',
        additional_env={'PYTHONPATH': _pythonpath},
        parameters=[{
            'waypoints_file': LaunchConfiguration('waypoints_file'),
            'nav_timeout':    5.0,
        }])

    return LaunchDescription([
        declare_waypoints,
        declare_model,
        nlp_node,
        agent_node,
    ])
