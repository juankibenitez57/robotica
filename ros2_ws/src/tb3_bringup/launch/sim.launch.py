import os
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Directorios ────────────────────────────────────────────────────────────
    pkg_bringup      = get_package_share_directory('tb3_bringup')
    pkg_tb3_sim      = get_package_share_directory('nav2_minimal_tb3_sim')
    pkg_tb3_desc     = get_package_share_directory('tb3_description')
    pkg_ros_gz       = get_package_share_directory('ros_gz_sim')
    pkg_irb120       = get_package_share_directory('irb120_jazzy_sim')

    # ── Argumentos configurables ───────────────────────────────────────────────
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

    # ── Variables de entorno para Gazebo ──────────────────────────────────────
    # Gazebo necesita saber dónde buscar los modelos del TB3
    gz_resource_tb3_models = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_tb3_sim, 'models'))

    gz_resource_tb3_parent = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(Path(pkg_tb3_sim).parent.resolve()))

    gz_irb120 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(Path(pkg_irb120).parent.resolve()))

    # ── 1. Gazebo Sim ──────────────────────────────────────────────────────────
    # -r arranca la simulación directamente sin pausa
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', world]}.items())

    # ── 2. Robot State Publisher — URDF combinado (TB3 + brazo) ───────────────
    arm_combined_urdf = os.path.join(pkg_tb3_desc, 'urdf', 'tb3_arm_combined.urdf.xacro')
    robot_description_content = subprocess.check_output(
        ['xacro', arm_combined_urdf]).decode('utf-8')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }])

    # ── 3. Spawn del robot en Gazebo (retrasado 3s) ────────────────────────────
    arm_sdf = os.path.join(pkg_tb3_desc, 'urdf', 'gz_waffle_arm.sdf.xacro')

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'turtlebot3_waffle',
            '-string', Command([
                FindExecutable(name='xacro'), ' ', arm_sdf,
                ' namespace:=',
            ]),
            '-x', x_pose, '-y', y_pose, '-z', '0.01',
        ])

    # ── 4. Bridge Gazebo ↔ ROS 2 ──────────────────────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_bringup, 'config', 'bridge.yaml'),
            'use_sim_time': True,
        }])

    # ── 5. Spawners ros2_control ───────────────────────────────────────────────
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        output='screen')

    spawn_arm_ctrl = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        output='screen')

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
        gz_resource_tb3_models,
        gz_resource_tb3_parent,
        gz_irb120,
        gazebo,
        robot_state_publisher,
        bridge,
        TimerAction(period=15.0, actions=[spawn_robot]),
        TimerAction(period=4.0,  actions=[spawn_jsb]),
        TimerAction(period=5.0,  actions=[spawn_arm_ctrl]),
        rviz,
    ])
