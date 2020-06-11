from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     # package name
        #     package="hardware_control",
        #     # namespace for the node, able to create multiple of the same with different namespaces
        #     node_namespace="arm_node",
        #     # executable should match the entry points console scripts node name from setup.py in the package
        #     # should match the install/package/lib/package/[name]
        #     node_executable="arm_node", 
        #     # name of the node
        #     node_name="arm_node"
        # )

        Node(
            package="hardware_control",
            node_executable="motor_pwm",
            node_name="motor_pwm",
            # parameters=["src/hardware_control/hw_params.yaml"]
        ),
        Node(
            package="hardware_control",
            node_executable="serial_comms",
            node_name="serial_comms",
            parameters=["src/hardware_control/hw_params.yaml"]
        )
        
    ])