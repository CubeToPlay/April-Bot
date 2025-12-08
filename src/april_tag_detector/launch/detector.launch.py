import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # AprilTag Detector Parameters
    tag_size_arg = DeclareLaunchArgument(
        'tag_size',
        default_value='0.16',
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
        default_value='',
        description='Directory containing AprilTag template images'
    )
    
    # Navigator Parameters
    approach_distance_arg = DeclareLaunchArgument(
        'approach_distance',
        default_value='0.5',
        description='Distance to approach AprilTag (meters)'
    )
    
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.2',
        description='Maximum linear speed (m/s)'
    )
    
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='0.5',
        description='Maximum angular speed (rad/s)'
    )
    
    waypoint_tolerance_arg = DeclareLaunchArgument(
        'waypoint_tolerance',
        default_value='0.3',
        description='Waypoint reaching tolerance (meters)'
    )
    
    tag_database_path_arg = DeclareLaunchArgument(
        'tag_database_path',
        default_value='discovered_tags.json',
        description='Path to tag database file'
    )
    
    frontier_grid_size_arg = DeclareLaunchArgument(
        'frontier_grid_size',
        default_value='0.5',
        description='Frontier exploration grid size (meters)'
    )
    
    cancel_command_arg = DeclareLaunchArgument(
        'cancel_command',
        default_value='11',
        description='Goal ID that cancels current navigation'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # AprilTag Detector Node
    detector_node = Node(
        package='april_tag_detector',
        executable='apriltag_detector',
        name='apriltag_detector',
        output='screen',
        parameters=[{
            'tag_size': LaunchConfiguration('tag_size'),
            'camera_topic': LaunchConfiguration('camera_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'tag_dir': LaunchConfiguration('tag_dir'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('/camera/image_raw', LaunchConfiguration('camera_topic')),
            ('/camera/camera_info', LaunchConfiguration('camera_info_topic')),
        ]
    )

    image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='camera_viewer',
        arguments=['/apriltag_detections_image'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        # Launch arguments
        tag_size_arg,
        camera_topic_arg,
        camera_info_topic_arg,
        publish_tf_arg,
        tag_dir_arg,
        approach_distance_arg,
        linear_speed_arg,
        angular_speed_arg,
        waypoint_tolerance_arg,
        tag_database_path_arg,
        frontier_grid_size_arg,
        cancel_command_arg,
        use_sim_time_arg,
        
        # Nodes
        detector_node,
        image_view_node,
    ])