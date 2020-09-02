from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    pid_node = Node(
        package="motion_control",
        node_executable="pid_controller",
        parameters=["src/motion_control/robot_params.yaml"]
    )

    diff_node = Node(
        package="motion_control",
        node_executable="diff_drive_controller",
        parameters=["src/motion_control/robot_params.yaml"]
    )

    motor_driver = Node(
        package="hardware_control",
        node_executable="motor_driver",
        parameters=["src/hardware_control/motor_params.yaml"]
    )

    ld.add_action(pid_node)
    ld.add_action(diff_node)
    ld.add_action(motor_driver)
    # ld.add_action(teleop)
    return ld