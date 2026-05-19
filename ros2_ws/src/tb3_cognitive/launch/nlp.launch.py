"""
nlp.launch.py — Lanza el nodo NLP cognitivo
--------------------------------------------

Usa el intérprete /opt/ai-venv/bin/python3 (via shebang del wrapper).
Inyecta explícitamente PYTHONPATH con paquetes IA + ROS2.

Uso:
  ros2 launch tb3_cognitive nlp.launch.py
  ros2 launch tb3_cognitive nlp.launch.py model:=facebook/bart-large-mnli

Verificar:
  ros2 topic echo /cognitive/command
  ros2 topic pub --once /nlp/input std_msgs/String "data: 've a la cocina'"
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Entorno IA separado de ROS2
_AI_VENV_SITE   = '/opt/ai-venv/lib/python3.12/site-packages'
_ROS2_SITE      = '/opt/ros/jazzy/lib/python3.12/site-packages'
_SYSTEM_PYTHONPATH = os.environ.get('PYTHONPATH', '')

# Construye PYTHONPATH explícito: venv primero (prioridad ABI), luego ROS2
_pythonpath = ':'.join(filter(None, [
    _AI_VENV_SITE,
    _SYSTEM_PYTHONPATH,
    _ROS2_SITE,
]))


def generate_launch_description():

    declare_model = DeclareLaunchArgument(
        'model',
        default_value='sentence-transformers/paraphrase-multilingual-MiniLM-L12-v2',
        description='Modelo sentence-transformers para clasificación semántica')

    nlp_node = Node(
        package='tb3_cognitive',
        executable='nlp_intent_node',
        name='nlp_intent_node',
        output='screen',
        # prefix hace que ROS2 ejecute: /opt/ai-venv/bin/python3 <script>
        # El shebang del archivo es irrelevante cuando se pasa prefix.
        # El venv Python tiene: numpy 2.x, sklearn 1.8, transformers, torch
        # PYTHONPATH inyecta: rclpy, tb3_msgs, tb3_cognitive
        prefix='/opt/ai-venv/bin/python3',
        additional_env={'PYTHONPATH': _pythonpath},
        parameters=[{
            'model':  LaunchConfiguration('model'),
            'device': -1,
        }])

    return LaunchDescription([
        declare_model,
        nlp_node,
    ])
