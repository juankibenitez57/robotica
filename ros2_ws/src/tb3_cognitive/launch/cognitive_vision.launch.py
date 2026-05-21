"""
cognitive_vision.launch.py — Capa cognitiva Fase 5: NLP + FSM + YOLO
----------------------------------------------------------------------

Lanza en orden:
  1. nlp_intent_node    — texto → /cognitive/command
  2. cognitive_fsm_node — FSM con goal tracking + detección visual
  3. yolo_detector_node — YOLOv8 → /detected_objects

Prerequisito:
  ros2 launch tb3_bringup navigation.launch.py  (con cámara RGB activa)

Uso:
  # Terminal 1
  ros2 launch tb3_bringup navigation.launch.py

  # Terminal 2
  ros2 launch tb3_cognitive cognitive_vision.launch.py

  # Terminal 3 — enviar comando de búsqueda
  ros2 topic pub --once /nlp/input std_msgs/String "data: 'busca una botella'"

  # Observar estado y detecciones
  ros2 topic echo /cognitive/fsm_state
  ros2 topic echo /detected_objects
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_AI_VENV_SITE     = '/opt/ai-venv/lib/python3.12/site-packages'
_SYSTEM_PYTHONPATH = os.environ.get('PYTHONPATH', '')
_pythonpath       = ':'.join(filter(None, [_AI_VENV_SITE, _SYSTEM_PYTHONPATH]))


def generate_launch_description():

    pkg = get_package_share_directory('tb3_cognitive')
    default_waypoints = os.path.join(pkg, 'config', 'waypoints.yaml')

    declare_waypoints = DeclareLaunchArgument(
        'waypoints_file',
        default_value=default_waypoints,
        description='Ruta al YAML con coordenadas de waypoints')

    declare_max_abs_coordinate = DeclareLaunchArgument(
        'max_abs_coordinate',
        default_value='20.0',
        description='Límite preventivo absoluto para x/y en frame map')

    declare_confidence = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.35',
        description='Confianza mínima NLP para ejecutar comando (0.0–1.0)')

    declare_max_nav_time = DeclareLaunchArgument(
        'max_nav_time',
        default_value='120.0',
        description='Watchdog: segundos máximos por goal antes de recovery')

    declare_max_retries = DeclareLaunchArgument(
        'max_retries',
        default_value='1',
        description='Reintentos automáticos tras recovery')

    declare_nlp_model = DeclareLaunchArgument(
        'nlp_model',
        default_value='sentence-transformers/paraphrase-multilingual-MiniLM-L12-v2',
        description='Modelo sentence-transformers para NLP')

    declare_yolo_model = DeclareLaunchArgument(
        'yolo_model',
        default_value='yolov8n.pt',
        description='Modelo YOLOv8 (yolov8n/s/m.pt o ruta local)')

    declare_yolo_confidence = DeclareLaunchArgument(
        'yolo_confidence',
        default_value='0.40',
        description='Umbral de confianza para detecciones YOLO (0.0–1.0); 0.40 en sim')

    nlp_node = Node(
        package='tb3_cognitive',
        executable='nlp_intent_node',
        name='nlp_intent_node',
        output='screen',
        prefix='/opt/ai-venv/bin/python3',
        additional_env={'PYTHONPATH': _pythonpath},
        parameters=[{
            'model':  LaunchConfiguration('nlp_model'),
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
            'waypoints_file':       LaunchConfiguration('waypoints_file'),
            'nav_timeout':          5.0,
            'max_abs_coordinate':   LaunchConfiguration('max_abs_coordinate'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'max_nav_time':         LaunchConfiguration('max_nav_time'),
            'max_retries':          LaunchConfiguration('max_retries'),
        }])

    yolo_node = Node(
        package='tb3_cognitive',
        executable='yolo_node',
        name='yolo_detector_node',
        output='screen',
        prefix='/opt/ai-venv/bin/python3',
        additional_env={'PYTHONPATH': _pythonpath},
        parameters=[{
            'model':      LaunchConfiguration('yolo_model'),
            'confidence': LaunchConfiguration('yolo_confidence'),
            'device':     -1,
        }])

    return LaunchDescription([
        declare_waypoints,
        declare_max_abs_coordinate,
        declare_confidence,
        declare_max_nav_time,
        declare_max_retries,
        declare_nlp_model,
        declare_yolo_model,
        declare_yolo_confidence,
        nlp_node,
        fsm_node,
        yolo_node,
    ])
