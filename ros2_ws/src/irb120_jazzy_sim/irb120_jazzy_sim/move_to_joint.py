import rclpy
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState


def main():
    rclpy.init()

    moveit = MoveItPy(node_name="irb120_moveit_py")
    arm = moveit.get_planning_component("irb120_arm")

    robot_model = moveit.get_robot_model()
    robot_state = RobotState(robot_model)

    joint_goal = {
        "joint_1": 0.0,
        "joint_2": -0.7,
        "joint_3": 0.9,
        "joint_4": 0.0,
        "joint_5": 1.0,
        "joint_6": 0.0,
    }

    robot_state.set_joint_group_positions("irb120_arm", joint_goal)

    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=robot_state)

    plan_result = arm.plan()

    if plan_result:
        moveit.execute(plan_result.trajectory)
        print("Trayectoria ejecutada")
    else:
        print("No se pudo planificar")

    rclpy.shutdown()


if __name__ == "__main__":
    main()