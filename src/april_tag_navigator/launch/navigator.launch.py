from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_dir = get_package_share_directory('april_tag_navigator')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_params_file = os.path.join(pkg_dir, 'config', 'slam_params.yaml')
    rviz_config_file = os.path.join(pkg_dir, 'config', 'nav_config.rviz') 

    navigator_node = Node(
        package='april_tag_navigator',
        executable='navigator',
        name='navigator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'approach_distance': 0.35,
            'linear_speed': 0.5,
            'angular_speed': 0.5,
            'waypoint_tolerance': 0.3,
            'frontier_grid_size': 0.5,
            'cancel_command': 11
        }]
    )

    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            slam_params_file,  # Use config file
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/scan')
        ]
    )

    # Automatically configure SLAM Toolbox on startup
    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    
    # Automatically activate SLAM Toolbox after configuration
    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )
    
    # Emit configure event when SLAM node starts
    emit_configure_on_start = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_toolbox_node,
            on_start=[configure_slam],
        )
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments= ['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        slam_toolbox_node,
        emit_configure_on_start,
        activate_slam,
        navigator_node,
        rviz_node,
    ])