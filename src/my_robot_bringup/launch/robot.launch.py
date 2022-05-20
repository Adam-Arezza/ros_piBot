from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    diff_drive_node = Node(
        package="motion_control",
        executable="diff_drive_controller",
        parameters=["src/motion_control/robot_params.yaml"]
    )

    pid_node = Node(
        package="motion_control",
        executable="pid_controller",
        parameters=["src/motion_control/robot_params.yaml"]
    )

    motor_driver_node = Node(
        package="hardware_control",
        executable="motor_driver",
        parameters=["src/hardware_control/motor_params.yaml"]
    )

    # serial_sensor_node = Node(
    #     package="hardware_control",
    #     node_executable="serial_comms"
    # )

    imu_node = Node(
        package="hardware_control",
        executable="imu_node"
    )

    velocity_node = Node(
        package="motion_control",
        executable="velocities",
        parameters=["src/motion_control/robot_params.yaml"]
    )

    static_frame = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '0', '0', 'world', 'base_link']
    )

    encoder_node = Node(
        package='hardware_control',
        executable='encoders'
    )

    pose_estimator_node = Node(
        package='motion_control',
        executable='pose_estimator',
        parameters=["src/motion_control/robot_params.yaml"]
    )

    base_tf_broadcaster = Node(
        package='motion_control',
        executable='base_tf_broadcaster'
    )

    ld.add_action(diff_drive_node)
    ld.add_action(pid_node)
    ld.add_action(motor_driver_node)
    # ld.add_action(serial_sensor_node)
    ld.add_action(imu_node)
    ld.add_action(velocity_node)
    ld.add_action(encoder_node)
    ld.add_action(pose_estimator_node)
    ld.add_action(base_tf_broadcaster)
    
    return ld