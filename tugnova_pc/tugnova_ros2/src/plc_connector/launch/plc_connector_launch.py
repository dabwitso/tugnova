from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="plc_connector",
            namespace="plc_connector",
            executable="plc_sender",
            name="plc_sender"
        ),
        Node(
            package="plc_connector",
            namespace="plc_connector",
            executable="plc_sender",
            name="plc_sender"
        )
    ])
