# racing_agent/launch/start_all.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the throttle_publisher node
        Node(
            package='racing_agent',
            executable='throttle_publisher',
            name='throttle_publisher',
            output='screen',
        ),

        # Start the localization node
        Node(
            package='racing_agent',  # Replace with the actual package name
            executable='localization',  # Replace with the actual node name
            name='localization',
            output='screen',
        ),

        # Start the mapping node
        # Node(
        #     package='racing_agent',  # Replace with the actual package name
        #     executable='mapping',  # Replace with the actual node name
        #     name='mapping',
        #     output='screen',
        # ),
    ])
