"""
cognitive_fsm.launch.py — Capa cognitiva Fase 3: NLP + FSM
-----------------------------------------------------------

Lanza en orden:
  1. nlp_intent_node  — texto → /cognitive/command
  2. cognitive_fsm_node — FSM con goal tracking → NAV2

Prerequisito:
  ros2 launch tb3_bringup navigation.launch.py

Uso:
  # Terminal 1
  ros2 launch tb3_bringup navigation.launch.py

  # Terminal 2
  ros2 launch tb3_cognitive cognitive_fsm.launch.py

  # Terminal 3 — enviar comando
  ros2 topic pub --once /nlp/input std_msgs/String "data: 've a la cocina'"

  # Observar estado FSM en tiempo real
  ros2 topic echo /cognitive/fsm_state
  ros2 topic echo /cognitive/status
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_AI_VENV_SITE    = '/opt/ai-venv/lib/python3.12/site-packages'
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

    fsm_node = Node(
        package='tb3_cognitive',
        executable='fsm_node',
        name='cognitive_fsm_node',
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
        fsm_node,
    ])
