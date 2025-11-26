from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hand_gestures',
            executable='webcam',
            name='webcam'
        ),
        Node(
            package='hand_gestures',
            executable='gestures',
            name='gestures'
        )
    ])
