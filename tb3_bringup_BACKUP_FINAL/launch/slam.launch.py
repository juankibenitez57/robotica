"""
slam.launch.py
--------------
Mapeo simultáneo con SLAM Toolbox (online async).

Incluye el launch oficial de slam_toolbox que maneja correctamente el ciclo
de vida del nodo (configure → activate). Sin esa gestión, el nodo arranca
pero nunca suscribe /scan ni publica /map.

Flujo de datos:
  Gazebo LiDAR → gz /scan → bridge → ROS /scan
  ROS /scan + TF (odom→base_footprint) → SLAM Toolbox → /map + TF map→odom

Para guardar el mapa cuando termines de explorar:
  ros2 run nav2_map_server map_saver_cli -f ~/map

Verificación:
  ros2 topic hz /scan          # ~1.7 Hz (normal con RTF 30%)
  ros2 topic hz /map           # ~0.2 Hz al moverse
  ros2 run tf2_tools view_frames  # debe aparecer map→odom→base_footprint
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_bringup    = get_package_share_directory('tb3_bringup')
    pkg_slam       = get_package_share_directory('slam_toolbox')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Lanzar RViz para visualizar el mapa')

    use_rviz = LaunchConfiguration('use_rviz')

    slam_params = os.path.join(
        pkg_bringup, 'config', 'mapper_params_online_async.yaml')

    # Usa el launch oficial de slam_toolbox que gestiona el lifecycle
    # (configure → activate) correctamente con autostart=true
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'true',
        }.items())

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(
            pkg_bringup, 'config', 'rviz', 'slam_view.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        declare_use_rviz,
        slam_launch,
        rviz,
    ])
