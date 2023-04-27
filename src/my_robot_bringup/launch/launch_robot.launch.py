from launch import LaunchDescription
from launch_ros.actions import Node
import os
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_description').find('robot_description')
xacro_file = os.path.join('src/robot_description/robot.urdf.xacro')
robot_description_config = xacro.process_file(xacro_file)
use_sim_time = LaunchConfiguration('use_sim_time')
params = {'robot_description': robot_description_config.toxml(), 'use_sim_time':use_sim_time}
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
        parameters=[params]
    )

    serial_node = Node(
        package='hardware_control',
        executable='serial_comms'
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
                'serial_port': '/dev/ttyUSB1',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }])
    
    laser_odom_node = Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/scan',
                    'odom_topic' : '/odom_rf2o',
                    'publish_tf' : False,
                    'base_frame_id' : 'base_link',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 5.0}],
            )
    
    map_to_odom_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node'
    )

    wheel_joint_pub = Node(
	package='motion_control',
	executable='wheel_joint_publisher'
    )

    joint_pub_node = Node(
	package='joint_state_publisher',
	executable='joint_state_publisher',
	parameters=[{'source_list':['wheel_states']}]
    )


    ld.add_action(diff_drive_node)
    ld.add_action(wheel_joint_pub)
    ld.add_action(joint_pub_node)
    ld.add_action(wheel_odometry_node)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(start_robot_localization_cmd)
    ld.add_action(laser_node)
    ld.add_action(laser_odom_node)
    ld.add_action(map_to_odom_static_tf)
    # ld.add_action(camera_node)
    ld.add_action(serial_node)
    
    return ld
