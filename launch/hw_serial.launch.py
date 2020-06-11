from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="hardware_control",
            node_namespace="nano_1",
            node_executable="serial_comms",
            node_name="serial_comms",
            parameters=["src/hardware_control/hw_params.yaml"]
        ),
        Node(
            package="hardware_control",
            node_namespace="nano_2",
            node_executable="serial_comms",
            node_name="serial_comms",
            parameters=["src/hardware_control/hw_params.yaml"]
        )
    ])