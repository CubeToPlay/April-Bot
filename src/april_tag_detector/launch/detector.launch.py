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
    tag_size_arg = DeclareLaunchArgument(
        'tag_size',
        default_value='0.5',
        description='Physical size of AprilTags in meters'
    )
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera image topic'
    )
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera_info',
        description='Camera info topic'
    )
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Publish TF transforms for detected tags'
    )
    
    tag_dir_arg = DeclareLaunchArgument(
        'tag_dir',
        default_value=tag_dir,
        description='Directory containing AprilTag template images'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # AprilTag Detector Node
    detector_node = Node(
        package='april_tag_detector',
        executable='detector',
        name='detector',
        output='screen',
        parameters=[{
            'tag_size': LaunchConfiguration('tag_size'),
            'camera_topic': LaunchConfiguration('camera_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'tag_dir': LaunchConfiguration('tag_dir'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    video_view_node = Node(
        package='april_tag_detector',
        executable='video_viewer',
        name='video_viewer',
        output='screen',
        parameters=[]
    )


    return LaunchDescription([
        # Launch arguments
        tag_size_arg,
        camera_topic_arg,
        camera_info_topic_arg,
        publish_tf_arg,
        tag_dir_arg,
        use_sim_time_arg,
        
        # Nodes
        detector_node,
        video_view_node,
    ])