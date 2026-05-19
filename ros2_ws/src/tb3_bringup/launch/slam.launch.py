"""
slam.launch.py
--------------
Paso 3a: Mapeo simultáneo con SLAM Toolbox (online async).

Lanza:
  - async_slam_toolbox_node → suscribe /scan + /tf
                              Publica: /map (OccupancyGrid)
                              Publica TF: map → odom
  - rviz2                   → visualiza mapa en tiempo real

Flujo de datos:
  Gazebo LiDAR → /scan (bridge) → SLAM Toolbox → /map + TF map→odom

Para guardar el mapa cuando el mapeo esté completo:
  ros2 run nav2_map_server map_saver_cli -f ~/maps/tb3_lab_map

Verificación:
  ros2 topic hz /scan          # debe ser ~5 Hz
  ros2 topic echo /map --once  # debe mostrar datos
  ros2 run tf2_tools view_frames  # debe mostrar map→odom→base_footprint
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_bringup = get_package_share_directory('tb3_bringup')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Lanzar RViz para visualizar el mapa')

    use_rviz = LaunchConfiguration('use_rviz')

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_bringup, 'config', 'mapper_params_online_async.yaml'),
            {'use_sim_time': True},
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(pkg_bringup, 'config', 'rviz', 'slam_view.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        declare_use_rviz,
        slam_toolbox,
        rviz,
    ])
