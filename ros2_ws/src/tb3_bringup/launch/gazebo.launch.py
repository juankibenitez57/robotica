"""
gazebo.launch.py
----------------
Paso 1 del stack de navegación autónoma.

Lanza:
  - Gazebo Harmonic con el mundo tb3_lab.sdf
  - robot_state_publisher  → publica /robot_description y TFs estáticos
                             (base_footprint→base_link→base_scan, etc.)
  - joint_state_publisher  → republica /joint_states
  - ros_gz_sim create      → spawna el TurtleBot3 en Gazebo
                             El plugin DiffDrive publica: odom→base_footprint
                             El plugin JointStatePublisher publica: /joint_states
  - ros_gz_bridge          → traduce topics GZ↔ROS2:
                             /clock, /cmd_vel, /odom, /tf, /scan, /imu, /joint_states

Cadena TF generada aquí:
  odom → base_footprint   (DiffDrive plugin en Gazebo, bridgeado vía /tf)
  base_footprint → base_link → base_scan, imu_link, camera_link
                              (robot_state_publisher desde URDF)
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Directorios ────────────────────────────────────────────────────────────
    pkg_bringup  = get_package_share_directory('tb3_bringup')
    pkg_tb3_sim  = get_package_share_directory('nav2_minimal_tb3_sim')
    pkg_tb3_desc = get_package_share_directory('tb3_description')
    pkg_ros_gz   = get_package_share_directory('ros_gz_sim')

    # ── Argumentos ────────────────────────────────────────────────────────────
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Lanzar RViz2')
    declare_world = DeclareLaunchArgument(
        'world', default_value=os.path.join(pkg_bringup, 'worlds', 'tb3_lab.sdf'),
        description='Ruta al fichero SDF del mundo')
    declare_x = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y = DeclareLaunchArgument('y_pose', default_value='0.0')

    use_rviz = LaunchConfiguration('use_rviz')
    world    = LaunchConfiguration('world')
    x_pose   = LaunchConfiguration('x_pose')
    y_pose   = LaunchConfiguration('y_pose')

    # ── Variables de entorno para modelos Gazebo ───────────────────────────────
    gz_models = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_tb3_sim, 'models'))
    gz_parent = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(Path(pkg_tb3_sim).parent.resolve()))

    # ── 1. Gazebo Harmonic ─────────────────────────────────────────────────────
    # -r  → arranca la simulación sin pausa
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', world]}.items())

    # ── 2. Robot State Publisher ───────────────────────────────────────────────
    # Publica /robot_description y calcula TFs desde el URDF
    # TFs publicados: base_footprint→base_link→{base_scan, imu_link, camera_link,
    #                  wheel_left_link, wheel_right_link, caster_*}
    robot_urdf = os.path.join(pkg_tb3_desc, 'urdf', 'tb3_waffle.urdf')
    with open(robot_urdf, 'r') as f:
        robot_description_content = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }])

    # ── 3. Joint State Publisher ──────────────────────────────────────────────
    # Agrega los estados de joints variables desde el bridge
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'source_list': ['/joint_states'],
        }])

    # ── 4. Spawn del robot en Gazebo ───────────────────────────────────────────
    robot_sdf = os.path.join(pkg_tb3_desc, 'urdf', 'gz_waffle.sdf.xacro')

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'turtlebot3_waffle',
            '-string', Command([
                FindExecutable(name='xacro'), ' ', robot_sdf,
                ' namespace:=',
            ]),
            '-x', x_pose, '-y', y_pose, '-z', '0.01',
        ])

    # ── 5. Bridge Gazebo Harmonic ↔ ROS 2 ────────────────────────────────────
    # Qué publica cada dirección:
    #   GZ→ROS: /clock, /odom, /tf, /scan, /imu, /joint_states
    #   ROS→GZ: /cmd_vel  (NAV2/teleop → DiffDrive en Gazebo)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_bringup, 'config', 'bridge.yaml'),
            'use_sim_time': True,
        }])

    # ── 6. RViz2 (opcional) ────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(pkg_bringup, 'config', 'rviz', 'tb3_view.rviz')],
        parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        declare_use_rviz,
        declare_world,
        declare_x,
        declare_y,
        gz_models,
        gz_parent,
        gazebo,
        robot_state_publisher,
        joint_state_pub,
        spawn_robot,
        bridge,
        rviz,
    ])
