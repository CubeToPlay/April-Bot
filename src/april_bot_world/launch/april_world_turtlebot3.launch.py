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
            package='april_bot_world',
            executable='odom_to_tf',
            name='odom_to_tf',
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