"""
bringup.launch.py
-----------------
Launcher maestro: arranca TODO el stack de navegación autónoma.

Modos disponibles (argumento 'nav_mode'):
  slam         → Gazebo + EKF + SLAM Toolbox + NAV2 + RViz
                 Usar para crear el mapa por primera vez.
  navigation   → Gazebo + EKF + Localización AMCL + NAV2 + RViz
                 Usar cuando ya tienes el mapa guardado.

Uso:
  # Modo SLAM (crear mapa):
  ros2 launch tb3_bringup bringup.launch.py nav_mode:=slam

  # Modo navegación (con mapa existente):
  ros2 launch tb3_bringup bringup.launch.py nav_mode:=navigation

  # Modo navegación con mapa específico:
  ros2 launch tb3_bringup bringup.launch.py nav_mode:=navigation \
    map:=/home/user/maps/mi_mapa.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    pkg_bringup = get_package_share_directory('tb3_bringup')
    launch_dir  = os.path.join(pkg_bringup, 'launch')

    # ── Argumentos ────────────────────────────────────────────────────────────
    declare_nav_mode = DeclareLaunchArgument(
        'nav_mode',
        default_value='slam',
        description='Modo de navegación: "slam" o "navigation"')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_bringup, 'maps', 'tb3_lab_map.yaml'),
        description='Mapa para modo navigation')

    declare_x = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y = DeclareLaunchArgument('y_pose', default_value='0.0')

    nav_mode = LaunchConfiguration('nav_mode')
    map_file = LaunchConfiguration('map')
    x_pose   = LaunchConfiguration('x_pose')
    y_pose   = LaunchConfiguration('y_pose')

    # ── 1. Gazebo ─────────────────────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'gazebo.launch.py')),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'use_rviz': 'false',
        }.items())

    # ── 2. EKF ────────────────────────────────────────────────────────────────
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'ekf.launch.py')))

    # ── 3a. SLAM (modo slam) ───────────────────────────────────────────────────
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'slam.launch.py')),
        condition=IfCondition(
            PythonExpression(["'", nav_mode, "' == 'slam'"])),
        launch_arguments={'use_rviz': 'false'}.items())

    # ── 3b. Localización AMCL (modo navigation) ────────────────────────────────
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'localization.launch.py')),
        condition=IfCondition(
            PythonExpression(["'", nav_mode, "' == 'navigation'"])),
        launch_arguments={'map': map_file}.items())

    # ── 4. NAV2 ───────────────────────────────────────────────────────────────
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'navigation.launch.py')))

    # ── 5. RViz2 ──────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_bringup, 'config', 'rviz', 'nav2.rviz')],
        parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        declare_nav_mode,
        declare_map,
        declare_x,
        declare_y,
        gazebo_launch,
        ekf_launch,
        slam_launch,
        localization_launch,
        navigation_launch,
        rviz,
    ])
