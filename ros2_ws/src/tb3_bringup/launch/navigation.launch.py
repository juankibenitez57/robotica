"""
navigation.launch.py
--------------------
Paso 4: Navegación autónoma con NAV2.

Lanza:
  - planner_server    → planificación global (NavFn / Dijkstra)
                        Suscribe: /map, pose actual
                        Publica:  /plan (global path)
  - controller_server → control local (DWB Controller)
                        Suscribe: /plan, /scan, /odom
                        Publica:  /cmd_vel → robot se mueve
                        Publica:  /local_plan
  - bt_navigator      → coordina planificador y controlador via Behavior Trees
                        Suscribe: /goal_pose (objetivo de navegación desde RViz)
  - behavior_server   → recuperación automática (spin, backup, wait)
  - velocity_smoother → suaviza las velocidades de /cmd_vel
  - waypoint_follower → navega por lista de waypoints
  - lifecycle_manager → gestiona el ciclo de vida de todos los nodos NAV2

Prerequisito:
  localization.launch.py o slam.launch.py corriendo
  (necesita TF map→odom activo)

Para enviar un objetivo desde RViz:
  Click en "2D Goal Pose" → click en el mapa

Para enviar objetivo por terminal:
  ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
    "{ header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5}}}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_bringup = get_package_share_directory('tb3_bringup')

    nav2_params = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': True}])

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel', '/cmd_vel')])

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': True}])

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': True}])

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': True}],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ])

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': True}])

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'planner_server',
                'controller_server',
                'bt_navigator',
                'behavior_server',
                'velocity_smoother',
                'waypoint_follower',
            ],
        }])

    return LaunchDescription([
        planner_server,
        controller_server,
        bt_navigator,
        behavior_server,
        velocity_smoother,
        waypoint_follower,
        lifecycle_manager,
    ])
