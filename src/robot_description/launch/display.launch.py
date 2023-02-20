import os
import launch_ros
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_description').find('robot_description')
    xacro_file = os.path.join('src/robot_description/robot.urdf.xacro')
    # default_rviz_config_path = os.path.join(pkg_share, 'src/rviz/urdf_config.rviz')
    robot_description_config = xacro.process_file(xacro_file)
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # joint_pub_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui'
    # )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', 'src/rviz/base_config.rviz']
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher,
        #joint_pub_node,
        #rviz_node
    ])
