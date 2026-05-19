"""
nlp.launch.py — Lanza solo la capa NLP (sin Gazebo, sin NAV2)
-------------------------------------------------------------

Uso:
  ros2 launch tb3_cognitive nlp.launch.py

  # Con modelo más ligero/pesado:
  ros2 launch tb3_cognitive nlp.launch.py model:=facebook/bart-large-mnli

Verificar:
  ros2 topic echo /cognitive/command
  ros2 topic pub /nlp/input std_msgs/String "data: 've a la cocina'"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    declare_model = DeclareLaunchArgument(
        'model',
        default_value='cross-encoder/nli-distilroberta-base',
        description='Modelo HuggingFace para zero-shot classification')

    nlp_node = Node(
        package='tb3_cognitive',
        executable='nlp_intent_node',
        name='nlp_intent_node',
        output='screen',
        parameters=[{
            'model':  LaunchConfiguration('model'),
            'device': -1,
        }])

    return LaunchDescription([
        declare_model,
        nlp_node,
    ])
