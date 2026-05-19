"""
navigation.launch.py
--------------------
Stack de navegación autónoma NAV2.

Prerrequisito: localization.launch.py corriendo (necesita TF map→odom).

Lifecycle nodes gestionados por lifecycle_manager_navigation:
  - controller_server  → DWB local planner, genera /cmd_vel
  - planner_server     → NavFn global planner, genera /plan
  - behavior_server    → recuperación (spin, backup, wait)
  - bt_navigator       → coordina todo con Behavior Trees
  - waypoint_follower  → navegación por lista de waypoints

Flujo de navegación:
  /goal_pose (RViz "2D Goal Pose")
    → bt_navigator
    → planner_server (/plan)
    → controller_server (/cmd_vel)
    → bridge → Gazebo DiffDrive → robot se mueve

Verificación:
  ros2 lifecycle get /planner_server    # active [4]
  ros2 lifecycle get /controller_server # active [4]
  ros2 lifecycle get /bt_navigator      # active [4]
  ros2 topic hz /plan                   # aparece al enviar un goal
  ros2 topic hz /local_plan             # aparece al navegar
  ros2 topic echo /cmd_vel              # velocidades al navegar

Problemas frecuentes:
  - "goal rejected": costmap vacío, scan no llega, o map frame no existe
  - robot no se mueve: /cmd_vel no llega al bridge → ver bridge.yaml
  - planner falla: mapa no cargado o robot fuera del mapa
  - controller oscila: reducir PathAlign.scale en nav2_params.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_bringup = get_package_share_directory('tb3_bringup')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')

    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params  = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')

    # ── controller_server ──────────────────────────────────────────────────────
    # DWB (Dynamic Window Based) local planner.
    # Suscribe: /local_costmap/costmap, /plan, /odom
    # Publica:  /cmd_vel, /local_plan
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}])

    # ── planner_server ─────────────────────────────────────────────────────────
    # NavFn global planner (Dijkstra o A*).
    # Suscribe: /map, /global_costmap/costmap
    # Publica:  /plan (nav_msgs/Path)
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}])

    # ── behavior_server ────────────────────────────────────────────────────────
    # Recuperaciones automáticas cuando el robot está atascado.
    # Behaviors: Spin (gira 360°), BackUp (retrocede), Wait (espera)
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}])

    # ── bt_navigator ───────────────────────────────────────────────────────────
    # Orquestador principal — usa Behavior Trees para coordinar
    # planner, controller y behaviors.
    # Recibe: /goal_pose (desde RViz o código)
    # Llama internamente a planner_server y controller_server
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}])

    # ── waypoint_follower ──────────────────────────────────────────────────────
    # Permite enviar una lista de waypoints para navegación multi-punto.
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}])

    # ── lifecycle_manager_navigation ───────────────────────────────────────────
    # Activa todos los nodos en orden. Si alguno falla, detiene la cadena.
    # Orden crítico: controller y planner ANTES de bt_navigator.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
            ],
        }])

    return LaunchDescription([
        declare_use_sim_time,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
    ])
