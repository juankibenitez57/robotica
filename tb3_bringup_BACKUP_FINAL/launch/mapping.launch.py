import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('tb3_bringup')

    # ── Argumentos ────────────────────────────────────────────────────────────
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Lanzar RViz para visualizar el mapa')

    use_rviz = LaunchConfiguration('use_rviz')

    # ── SLAM Toolbox (modo online asíncrono) ──────────────────────────────────
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml'),
            {'use_sim_time': True}
        ],
    )

    # ── RViz para monitorizar el mapa ─────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'config', 'rviz', 'slam_view.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        declare_use_rviz,
        slam_toolbox,
        rviz,
    ])
