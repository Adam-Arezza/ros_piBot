from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    diff_drive_node = Node(
        package="motion_control",
        node_executable="diff_drive_controller",
        parameters=["src/motion_control/robot_params.yaml"]
    )

    pid_node = Node(
        package="motion_control",
        node_executable="pid_controller",
        parameters=["src/motion_control/robot_params.yaml"]
    )

    motor_driver_node = Node(
        package="hardware_control",
        node_executable="motor_driver",
        parameters=["src/hardware_control/motor_params.yaml"]
    )

    serial_sensor_node = Node(
        package="hardware_control",
        node_executable="serial_comms"
    )

    imu_node = Node(
        package="hardware_control",
        node_executable="imu_node"
    )

    wheel_speed_node = Node(
        package="motion_control",
        node_executable="wheel_speed",
        parameters=["src/motion_control/robot_params.yaml"]
    )

    ld.add_action(diff_drive_node)
    ld.add_action(pid_node)
    ld.add_action(motor_driver_node)
    ld.add_action(serial_sensor_node)
    ld.add_action(imu_node)
    ld.add_action(wheel_speed_node)
    
    return ld