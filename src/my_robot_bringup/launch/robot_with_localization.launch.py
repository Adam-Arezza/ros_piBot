from launch import LaunchDescription
from launch_ros.actions import Node
import os
import launch_ros
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_description').find('robot_description')
xacro_file = os.path.join('src/robot_description/robot_description.urdf.xacro')
robot_description_config = xacro.process_file(xacro_file)

def generate_launch_description():
    ld = LaunchDescription()

    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=["config/ekf.yaml"])

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config.toxml()}]
    )

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

    imu_driver = Node(
        package="hardware_control",
        executable="imu_driver"
    )

    # static_frame = Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         arguments = ['0', '0', '0', '0', '0', '0', 'world', 'base_link']
    # )

    encoder_node = Node(
        package='hardware_control',
        executable='encoders'
    )

    wheel_odometry_node = Node(
        package='motion_control',
        executable='wheel_odometry',
        parameters=["src/motion_control/robot_params.yaml"]
    )

    laser_node = Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }])

    # base_tf_broadcaster = Node(
    #     package='motion_control',
    #     executable='base_tf_broadcaster'
    # )

    ld.add_action(diff_drive_node)
    ld.add_action(pid_node)
    ld.add_action(motor_driver_node)
    ld.add_action(imu_driver)
    ld.add_action(encoder_node)
    ld.add_action(wheel_odometry_node)
    # ld.add_action(base_tf_broadcaster)
    ld.add_action(node_robot_state_publisher)
    # ld.add_action(static_frame)
    ld.add_action(start_robot_localization_cmd)
    ld.add_action(laser_node)
    
    return ld