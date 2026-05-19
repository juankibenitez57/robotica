"""
localization.launch.py
----------------------
Localización con mapa preexistente (AMCL + map_server).

Lanza como lifecycle nodes gestionados por lifecycle_manager_localization:
  - map_server  → carga el .pgm/.yaml, publica /map (Transient Local)
  - amcl        → filtro de partículas sobre /scan + TF
                  Publica: /amcl_pose, /particle_cloud
                  Publica TF: map → odom

Prerequisito:
  Gazebo corriendo con bridge activo (/scan y /tf disponibles).

Verificación:
  ros2 lifecycle get /map_server   # debe decir "active [4]"
  ros2 lifecycle get /amcl         # debe decir "active [4]"
  ros2 topic hz /map               # debe publicar (Transient Local, una vez)
  ros2 topic hz /amcl_pose         # debe publicar al mover el robot
  ros2 run tf2_ros tf2_echo map odom  # debe mostrar transform

Problemas frecuentes:
  - map_server inactive: lifecycle_manager no lo activó → ver logs
  - amcl no converge: initial_pose incorrecta → usar 2D Pose Estimate en RViz
  - map frame no existe: amcl inactive o scan no llega → ros2 topic hz /scan
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_bringup = get_package_share_directory('tb3_bringup')

    # ── Argumentos ─────────────────────────────────────────────────────────────
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_bringup, 'maps', 'house_map.yaml'),
        description='Ruta absoluta al YAML del mapa')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usar reloj de simulación de Gazebo')

    map_file      = LaunchConfiguration('map')
    use_sim_time  = LaunchConfiguration('use_sim_time')

    amcl_yaml    = os.path.join(pkg_bringup, 'config', 'amcl.yaml')
    nav2_params  = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')

    # ── map_server ─────────────────────────────────────────────────────────────
    # Carga el mapa PGM/YAML y publica /map con QoS Transient Local.
    # Es un lifecycle node: necesita configure+activate del lifecycle_manager.
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params,
            {
                'use_sim_time': use_sim_time,
                'yaml_filename': map_file,
            },
        ])

    # ── amcl ───────────────────────────────────────────────────────────────────
    # Filtro de partículas Monte Carlo para localización en el mapa.
    # Suscribe: /scan, /tf (odom→base_footprint)
    # Publica:  /amcl_pose (pose con covarianza), /particle_cloud (nube)
    # Publica TF: map → odom (la más importante)
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_yaml,
            {'use_sim_time': use_sim_time},
        ])

    # ── lifecycle_manager_localization ─────────────────────────────────────────
    # Gestiona el ciclo de vida de map_server y amcl en orden:
    #   unconfigured → configuring → inactive → activating → active
    # Con autostart=true lo hace automáticamente al arrancar.
    # Orden de activación: primero map_server (necesita publicar /map
    # antes de que amcl lo use).
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }])

    return LaunchDescription([
        declare_map,
        declare_use_sim_time,
        map_server,
        amcl,
        lifecycle_manager,
    ])
