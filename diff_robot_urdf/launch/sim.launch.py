import os
import tempfile

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    package_name = 'diff_robot_urdf'
    pkg_share = get_package_share_directory(package_name)
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')

    xacro_file = os.path.join(pkg_share, 'urdf', 'tracer_v1.xacro')
    bridge_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')
    
    urdf_tmp = os.path.join(tempfile.gettempdir(), 'tracer_v1.urdf')
    os.system(f"xacro {xacro_file} -o {urdf_tmp}")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world}'}.items()
    )

    robot_description = Command(['xacro ', xacro_file])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'tracer',
            '-file', urdf_tmp,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3'
        ]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config
        }]
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'source_list': ['/joint_states']
        }]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    return [gz_sim, rsp, joint_state_pub, spawn_robot, bridge, rviz]


def generate_launch_description():
    pkg_share = get_package_share_directory('diff_robot_urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'tracer_classroom.sdf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Usar tiempo de simulación'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='Ruta al world'
        ),
        OpaqueFunction(function=launch_setup)
    ])