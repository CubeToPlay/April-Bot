import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

tag_dir = os.path.join(
        get_package_share_directory('apriltag_resources'),
        'models'
    )

def generate_launch_description():
    # AprilTag Detector Parameters
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            output='screen',
            parameters=[{
                'image_transport': 'raw',
                'family': '36h11',
                'size': 0.16,  # meters (match your tag_size)
                'max_hamming': 2,
                'publish_tf': True,
                'camera_frame': 'camera_link',
                'approximate_sync': True,
                'queue_size': 10
            }],
            remappings=[
                ('image_rect', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info')
            ]
        ),
        Node(
            package='april_tag_detector',
            executable='detector',
            name='detector',
            output='screen',
            parameters=[]
        ),
        Node(
            package='april_tag_detector',
            executable='video_viewer',
            name='video_viewer',
            output='screen',
            parameters=[]
        )
    ])