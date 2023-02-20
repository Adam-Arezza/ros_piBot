import os
# import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
# import xacro
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_description').find('robot_description')
    xacro_file = os.path.join('src/robot_description/robot.urdf.xacro')
    # default_rviz_config_path = os.path.join(pkg_share, 'src/rviz/urdf_config.rviz')
    # robot_description_config = xacro.process_file(xacro_file)

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description_config = Command(['xacro ', xacro_file, ' sim_mode:=', use_sim_time])
    gazebo_params_file = 'config/gazebo_params.yaml'

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                                       get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                                       launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )

    spawn_entity = Node(package='gazebo_ros',
                        executable='spawn_entity.py',
                        arguments=['-topic', '/robot_description', '-entity', 'pibot'],
                        output='screen')
    diff_cont = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['diff_cont']
    )

    joint_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=['joint_broad']
    )



    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),

        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        diff_cont,
        joint_broadcaster_spawner
    ])