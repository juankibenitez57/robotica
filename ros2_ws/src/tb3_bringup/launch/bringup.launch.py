"""
bringup.launch.py — Launcher maestro TB3 Navigation Stack
----------------------------------------------------------

Modos:
  slam        → Gazebo + SLAM Toolbox + RViz (construir mapa)
  navigation  → Stack completo autónomo via navigation.launch.py

Uso:
  # Mapear:
  ros2 launch tb3_bringup bringup.launch.py nav_mode:=slam

  # Navegar con el mapa por defecto (house_map.yaml):
  ros2 launch tb3_bringup bringup.launch.py nav_mode:=navigation

  # Navegar con otro mapa:
  ros2 launch tb3_bringup bringup.launch.py nav_mode:=navigation \\
    map:=/ruta/al/mi_mapa.yaml

Workflow completo:
  1. slam mode  → mover robot con teleop hasta cubrir el área
  2. Guardar mapa:
       ros2 run nav2_map_server map_saver_cli \\
         -f ~/code/ROBOTICA/ros2_ws/src/tb3_bringup/maps/mi_mapa
  3. navigation mode con map:=.../mi_mapa.yaml
  4. En RViz: "2D Pose Estimate" para inicializar AMCL
  5. En RViz: "2D Goal Pose" para enviar objetivos
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    pkg_bringup = get_package_share_directory('tb3_bringup')
    launch_dir  = os.path.join(pkg_bringup, 'launch')

    # ── Argumentos ─────────────────────────────────────────────────────────────
    declare_nav_mode = DeclareLaunchArgument(
        'nav_mode',
        default_value='slam',
        description='"slam" para mapear | "navigation" para navegar')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_bringup, 'maps', 'house_map.yaml'),
        description='Mapa YAML para modo navigation')

    declare_x = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y = DeclareLaunchArgument('y_pose', default_value='0.0')

    nav_mode = LaunchConfiguration('nav_mode')
    map_file = LaunchConfiguration('map')
    x_pose   = LaunchConfiguration('x_pose')
    y_pose   = LaunchConfiguration('y_pose')

    is_slam = PythonExpression(["'", nav_mode, "' == 'slam'"])
    is_nav  = PythonExpression(["'", nav_mode, "' == 'navigation'"])

    # ── SLAM mode: Gazebo + SLAM Toolbox + RViz ───────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'gazebo.launch.py')),
        condition=IfCondition(is_slam),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'use_rviz': 'false',
        }.items())

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'slam.launch.py')),
        condition=IfCondition(is_slam),
        launch_arguments={'use_rviz': 'false'}.items())

    rviz_slam = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(is_slam),
        arguments=['-d', os.path.join(
            pkg_bringup, 'config', 'rviz', 'slam_view.rviz')],
        parameters=[{'use_sim_time': True}])

    # ── Navigation mode: navigation.launch.py incluye todo ───────────────────
    # (Gazebo + localization + NAV2 + RViz)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'navigation.launch.py')),
        condition=IfCondition(is_nav),
        launch_arguments={
            'map': map_file,
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items())

    return LaunchDescription([
        declare_nav_mode,
        declare_map,
        declare_x,
        declare_y,
        gazebo_launch,
        slam_launch,
        rviz_slam,
        navigation_launch,
    ])
