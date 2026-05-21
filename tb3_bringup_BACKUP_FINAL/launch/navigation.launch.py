"""
navigation.launch.py — Launcher completo de navegación autónoma
---------------------------------------------------------------

Lanzamiento único para navegación autónoma con TurtleBot3.

Uso:
  ros2 launch tb3_bringup navigation.launch.py

  # Con otro mapa:
  ros2 launch tb3_bringup navigation.launch.py map:=/ruta/mapa.yaml

Lanza en orden:
  1. Gazebo Harmonic  (robot + bridge + RSP)
  2. map_server       (carga house_map.yaml, publica /map)
  3. amcl             (localiza con /scan, publica TF map→odom)
  4. lifecycle_manager_localization  (activa map_server + amcl)
  5. planner_server   (NavFn, genera /plan)
  6. controller_server (DWB, genera /cmd_vel)
  7. bt_navigator     (coordina con Behavior Trees)
  8. behavior_server  (spin, backup, wait)
  9. waypoint_follower
 10. lifecycle_manager_navigation (activa todo el stack NAV2)
 11. RViz2 con nav2.rviz

Workflow:
  1. Esperar a que Gazebo y RViz carguen (~30 seg)
  2. Verificar lifecycles activos:
       ros2 lifecycle get /map_server    # active [4]
       ros2 lifecycle get /amcl          # active [4]
       ros2 lifecycle get /bt_navigator  # active [4]
  3. En RViz → "2D Pose Estimate" → click donde está el robot en el mapa
     (AMCL inicializa las partículas en esa posición)
  4. En RViz → "2D Goal Pose" → click en el destino
     (el robot planifica y navega autónomamente)

Diagnóstico:
  ros2 topic hz /scan              # ~1.7 Hz (normal con RTF 30%)
  ros2 topic hz /map               # Transient Local, 1 sola publicación
  ros2 topic hz /amcl_pose         # ~0.5 Hz al moverse
  ros2 topic hz /plan              # aparece al recibir goal
  ros2 topic hz /cmd_vel           # al navegar
  ros2 run tf2_tools view_frames   # map→odom→base_footprint→base_link→base_scan
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_bringup = get_package_share_directory('tb3_bringup')
    launch_dir  = os.path.join(pkg_bringup, 'launch')

    # ── Argumentos ─────────────────────────────────────────────────────────────
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_bringup, 'maps', 'house_map.yaml'),
        description='Ruta al YAML del mapa')

    declare_x = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y = DeclareLaunchArgument('y_pose', default_value='0.0')

    map_file = LaunchConfiguration('map')
    x_pose   = LaunchConfiguration('x_pose')
    y_pose   = LaunchConfiguration('y_pose')

    nav2_params = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')

    # ── 1. Gazebo + bridge + RSP ───────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'gazebo.launch.py')),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'use_rviz': 'false',
        }.items())

    # ── 2 + 3 + 4. map_server + amcl + lifecycle_manager_localization ─────────
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'localization.launch.py')),
        launch_arguments={'map': map_file}.items())

    # ── 5. planner_server ──────────────────────────────────────────────────────
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': True}])

    # ── 6. controller_server ───────────────────────────────────────────────────
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': True}])

    # ── 7. bt_navigator ────────────────────────────────────────────────────────
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': True}])

    # ── 8. behavior_server ─────────────────────────────────────────────────────
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': True}])

    # ── 9. waypoint_follower ───────────────────────────────────────────────────
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': True}])

    # ── 10. lifecycle_manager_navigation ───────────────────────────────────────
    # Orden de activación: controller y planner antes de bt_navigator
    lifecycle_manager_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
            ],
        }])

    # ── 11. RViz2 ──────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_bringup, 'rviz', 'nav2.rviz')],
        parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        declare_map,
        declare_x,
        declare_y,
        gazebo_launch,
        localization_launch,
        planner_server,
        controller_server,
        bt_navigator,
        behavior_server,
        waypoint_follower,
        lifecycle_manager_nav,
        rviz,
    ])
