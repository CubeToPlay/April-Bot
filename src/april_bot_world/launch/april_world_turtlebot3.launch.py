from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('april_bot_world')
    pkg_path = FindPackageShare('april_bot_world') #current package
    world_path = os.path.join(pkg_share, 'worlds', 'apriltag_world.sdf')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=(
                os.environ.get('GZ_SIM_RESOURCE_PATH', '') +
                ':' +
                os.path.join(
                    get_package_share_directory('apriltag_resources'),
                    'models'
                )
            )
        ),
        
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            PathJoinSubstitution([pkg_path, 'models'])
        ),
        
        SetEnvironmentVariable(
            'TURTLEBOT3_MODEL',
            'waffle'
        ),
        # Launch Gazebo with your world
        ExecuteProcess(
            cmd=['gz', 'sim', world_path, '-r'],
            output='screen',
            shell=False
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': open("/turtlebot3_waffle_gz/model.sdf").read()
            }]
        ),
        # Static transform: base_footprint -> base_scan
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_scan',
            output='screen',
            arguments=['--x', '0', '--y', '0', '--z', '0.172', 
                      '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                      '--frame-id', 'base_footprint', '--child-frame-id', 'base_scan'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_rgb_optical_tf',
            arguments=[
                '0', '0', '0',          # translation
                '-1.5708', '0', '-1.5708',  # roll pitch yaw
                'camera_rgb_frame',    # parent
                'camera_rgb_optical_frame'  # child
            ]
        ),
        
        # Static transform: base_footprint -> camera_rgb_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_camera',
            output='screen',
            arguments=['--x', '0.073', '--y', '-0.011', '--z', '0.084',
                      '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', 
                      '--frame-id', 'base_footprint', '--child-frame-id', 'camera_rgb_frame'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='april_bot_world',
            executable='odom_to_tf',
            name='odom_to_tf',
            parameters=[{
                'use_sim_time': True,
            }],
            output='screen'
        ),
        Node(
            package='april_bot_world',
            executable='scan_inf_fixer',
            name='scan_inf_fixer',
            parameters=[{
                'use_sim_time': True,
            }],
            output='screen'
        ),
        # Bridging and remapping Gazebo topics to ROS 2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            ],
            remappings=[
                ('/camera/image_raw', '/camera/image_raw'),
            ],
            parameters=[{
                'use_sim_time': True,
                'qos_overrides./scan.reliability': 'best_effort',
            }],
            output='screen'
        ),
    ])