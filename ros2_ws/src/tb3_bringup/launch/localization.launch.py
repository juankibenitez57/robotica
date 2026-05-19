"""
localization.launch.py
----------------------
Paso 3b: Localización con mapa preexistente (AMCL).
Usar DESPUÉS de haber guardado el mapa con SLAM.

Lanza:
  - map_server   → carga el mapa .pgm/.yaml desde disco
                   Publica: /map (OccupancyGrid estático)
  - amcl         → localiza el robot en el mapa con filtro de partículas
                   Suscribe: /scan, /odom, /map
                   Publica:  /amcl_pose, /particle_cloud
                   Publica TF: map → odom
  - lifecycle_manager → gestiona el ciclo de vida de map_server y amcl

Flujo de datos:
  /map (map_server) + /scan + /odom → AMCL → TF map→odom + /amcl_pose

Prerequisito:
  Tener un mapa guardado en tb3_bringup/maps/tb3_lab_map.yaml

Verificación:
  ros2 topic echo /amcl_pose --once
  ros2 topic echo /particle_cloud --once
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_bringup = get_package_share_directory('tb3_bringup')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_bringup, 'maps', 'tb3_lab_map.yaml'),
        description='Ruta al fichero YAML del mapa')

    map_file = LaunchConfiguration('map')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_file,
        }])

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            os.path.join(pkg_bringup, 'config', 'nav2_params.yaml'),
            {'use_sim_time': True},
        ])

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }])

    return LaunchDescription([
        declare_map,
        map_server,
        amcl,
        lifecycle_manager,
    ])
