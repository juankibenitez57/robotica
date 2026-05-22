"""
moveit_arm.launch.py — MoveIt move_group + RViz para control manual del brazo
------------------------------------------------------------------------------

Lanzar DESPUÉS de navigation_arm.launch.py (necesita Gazebo + RSP + controllers).

Uso:
  # Terminal 1
  ros2 launch tb3_bringup navigation_arm.launch.py

  # Terminal 2 (esperar ~15s a que Gazebo + controllers estén activos)
  ros2 launch tb3_bringup moveit_arm.launch.py

En RViz:
  1. Add → MotionPlanning (desde el panel Displays)
  2. En "Planning Group" seleccionar: irb120_arm
  3. Arrastrar el marcador interactivo (end-effector) al destino
  4. "Plan" → "Execute"

Diagnóstico:
  ros2 control list_controllers          # arm_controller active
  ros2 action list                       # /arm_controller/follow_joint_trajectory
  ros2 topic echo /move_group/status    # estado del move_group
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    pkg_bringup  = get_package_share_directory('tb3_bringup')
    pkg_tb3_desc = get_package_share_directory('tb3_description')

    combined_urdf_xacro      = os.path.join(pkg_tb3_desc, 'urdf', 'tb3_arm_combined.urdf.xacro')
    tb3_arm_srdf             = os.path.join(pkg_bringup, 'config', 'tb3_arm.srdf')
    arm_moveit_controllers   = os.path.join(pkg_bringup, 'config', 'arm_moveit_controllers.yaml')
    arm_kinematics           = os.path.join(pkg_bringup, 'config', 'arm_kinematics.yaml')

    moveit_config = (
        MoveItConfigsBuilder("tb3_arm", package_name="tb3_bringup")
        .robot_description(file_path=combined_urdf_xacro)
        .robot_description_semantic(file_path=tb3_arm_srdf)
        .trajectory_execution(file_path=arm_moveit_controllers)
        .planning_pipelines(pipelines=["ompl"])
        .robot_description_kinematics(file_path=arm_kinematics)
        .to_moveit_configs()
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
        ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_bringup, 'config', 'rviz', 'moveit_arm.rviz')],
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
        ])

    return LaunchDescription([
        move_group,
        TimerAction(period=3.0, actions=[rviz]),
    ])
