"""
navigation_arm.launch.py — NAV2 completo + brazo IRB120
-------------------------------------------------------

Igual que navigation.launch.py pero usa gazebo_arm.launch.py en lugar de
gazebo.launch.py. Todo el stack NAV2 (AMCL, planners, BT, RViz) es idéntico.

Uso:
  # Terminal 1 — NAV2 + brazo
  ros2 launch tb3_bringup navigation_arm.launch.py

  # Terminal 2 — Capa cognitiva con YOLO
  ros2 launch tb3_cognitive cognitive_vision.launch.py

  # Verificar TF del brazo
  ros2 run tf2_tools view_frames
  ros2 topic echo /joint_states  # debe mostrar wheel + arm joints

  # Mover brazo manualmente
  ros2 topic pub /arm_controller/joint_trajectory ...
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

    # ── 1. Gazebo + bridge + RSP (con brazo) ──────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'gazebo_arm.launch.py')),
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
