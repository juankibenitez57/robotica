"""
bringup.launch.py
-----------------
Launcher maestro: arranca TODO el stack de navegación autónoma.

Modos disponibles (argumento 'nav_mode'):
  slam         → Gazebo + SLAM Toolbox + RViz
                 Usar para crear el mapa por primera vez.
                 Mueve el robot con teleop para construir el mapa.
                 Guarda el mapa con:
                   ros2 run nav2_map_server map_saver_cli -f ~/map

  navigation   → Gazebo + Localización AMCL + NAV2 + RViz
                 Usar cuando ya tienes el mapa guardado.
                 Envía objetivos con el botón "2D Goal Pose" en RViz.

Uso:
  # Modo SLAM (crear mapa):
  ros2 launch tb3_bringup bringup.launch.py nav_mode:=slam

  # Modo navegación (con mapa existente):
  ros2 launch tb3_bringup bringup.launch.py nav_mode:=navigation

  # Modo navegación con mapa específico:
  ros2 launch tb3_bringup bringup.launch.py nav_mode:=navigation \\
    map:=/home/user/maps/mi_mapa.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
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

    is_slam = PythonExpression(["'", nav_mode, "' == 'slam'"])
    is_nav  = PythonExpression(["'", nav_mode, "' == 'navigation'"])

    # ── 1. Gazebo (siempre) ───────────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'gazebo.launch.py')),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'use_rviz': 'false',
        }.items())

    # ── 2a. SLAM Toolbox (solo en modo slam) ──────────────────────────────────
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'slam.launch.py')),
        condition=IfCondition(is_slam),
        launch_arguments={'use_rviz': 'false'}.items())

    # ── 2b. Localización AMCL (solo en modo navigation) ──────────────────────
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'localization.launch.py')),
        condition=IfCondition(is_nav),
        launch_arguments={'map': map_file}.items())

    # ── 3. NAV2 (solo en modo navigation) ────────────────────────────────────
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'navigation.launch.py')),
        condition=IfCondition(is_nav))

    # ── 4. RViz2 ──────────────────────────────────────────────────────────────
    # Modo slam: usa slam_view.rviz (vista TopDown optimizada para mapeo)
    # Modo nav:  usa nav2.rviz (con costmaps, paths, AMCL particles)
    rviz_slam = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(is_slam),
        arguments=['-d', os.path.join(pkg_bringup, 'config', 'rviz', 'slam_view.rviz')],
        parameters=[{'use_sim_time': True}])

    rviz_nav = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(is_nav),
        arguments=['-d', os.path.join(pkg_bringup, 'config', 'rviz', 'nav2.rviz')],
        parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        declare_nav_mode,
        declare_map,
        declare_x,
        declare_y,
        gazebo_launch,
        slam_launch,
        localization_launch,
        navigation_launch,
        rviz_slam,
        rviz_nav,
    ])
