# apriltag_navigator_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments (can be overridden at runtime)
    approach_distance_arg = DeclareLaunchArgument(
        'approach_distance', default_value='0.5',
        description='How close the robot must get to the tag to consider it approached'
    )
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed', default_value='0.2',
        description='Max linear speed (m/s)'
    )
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed', default_value='0.5',
        description='Max angular speed (rad/s)'
    )
    waypoint_tolerance_arg = DeclareLaunchArgument(
        'waypoint_tolerance', default_value='0.3',
        description='Waypoint acceptance radius (m)'
    )
    tag_database_path_arg = DeclareLaunchArgument(
        'tag_database_path', default_value='discovered_tags.json',
        description='Path to saved tag database file'
    )
    frontier_grid_size_arg = DeclareLaunchArgument(
        'frontier_grid_size', default_value='0.5',
        description='Resolution of exploration grid (m)'
    )
    cancel_command_arg = DeclareLaunchArgument(
        'cancel_command', default_value='11',
        description='Goal id value used to cancel current mission'
    )

    apriltag_navigator_node = Node(
        package='april_tag_navigator',
        executable='navigator',
        name='navigator',
        output='screen',
        parameters=[{
            'approach_distance': LaunchConfiguration('approach_distance'),
            'linear_speed': LaunchConfiguration('linear_speed'),
            'angular_speed': LaunchConfiguration('angular_speed'),
            'waypoint_tolerance': LaunchConfiguration('waypoint_tolerance'),
            'tag_database_path': LaunchConfiguration('tag_database_path'),
            'frontier_grid_size': LaunchConfiguration('frontier_grid_size'),
            'cancel_command': LaunchConfiguration('cancel_command'),
        }],
        emulate_tty=True
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    ld = LaunchDescription()

    # Add declared arguments so they show up for ros2 launch overrides
    ld.add_action(approach_distance_arg)
    ld.add_action(linear_speed_arg)
    ld.add_action(angular_speed_arg)
    ld.add_action(waypoint_tolerance_arg)
    ld.add_action(tag_database_path_arg)
    ld.add_action(frontier_grid_size_arg)
    ld.add_action(cancel_command_arg)

    # Add node
    ld.add_action(apriltag_navigator_node)
    ld.add_action(rviz_node)

    return ld
