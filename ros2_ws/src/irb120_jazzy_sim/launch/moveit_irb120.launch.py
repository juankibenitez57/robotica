from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("irb120_jazzy_sim"))

    urdf_xacro = str(pkg_share / "urdf" / "irb120.urdf.xacro")

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                urdf_xacro,
                " use_gazebo:=false",
            ]),
            value_type=str,
        )
    }

    moveit_config = (
        MoveItConfigsBuilder("irb120", package_name="irb120_jazzy_sim")
        .robot_description(file_path=urdf_xacro)
        .robot_description_semantic(file_path=str(pkg_share / "config" / "irb120.srdf"))
        .trajectory_execution(file_path=str(pkg_share / "config" / "moveit_controllers.yaml"))
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path=str(pkg_share / "config" / "joint_limits.yaml"))
        .robot_description_kinematics(file_path=str(pkg_share / "config" / "kinematics.yaml"))
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            robot_description,
            {"use_sim_time": True},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            robot_description,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        move_group,
        rviz,
    ])