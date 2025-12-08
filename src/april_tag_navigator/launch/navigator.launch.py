import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
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

    navigator_node = Node(
        package='april_tag_navigator',
        executable='apriltag_navigator',
        name='apriltag_navigator',
        output='screen',
        parameters=[{
            'approach_distance': LaunchConfiguration('approach_distance'),
            'linear_speed': LaunchConfiguration('linear_speed'),
            'angular_speed': LaunchConfiguration('angular_speed'),
            'waypoint_tolerance': LaunchConfiguration('waypoint_tolerance'),
            'tag_database_path': LaunchConfiguration('tag_database_path'),
            'frontier_grid_size': LaunchConfiguration('frontier_grid_size'),
            'cancel_command': LaunchConfiguration('cancel_command'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('/scan', '/scan'),
            ('/map', '/map'),
            ('/goal_id', '/goal_id'),
            ('/april_tags', '/april_tags'),
            ('/cmd_vel', '/cmd_vel'),
        ]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '$(find rviz2)/default.rviz'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        # Launch arguments
        approach_distance_arg,
        linear_speed_arg,
        angular_speed_arg,
        waypoint_tolerance_arg,
        tag_database_path_arg,
        frontier_grid_size_arg,
        cancel_command_arg,
        use_sim_time_arg,
        
        # Nodes
        navigator_node,
        rviz_node,
    ])