from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim') #ros/gazebo package
    turtlebot3_gazebo_pkg_path = get_package_share_directory('turtlebot3_gazebo') #turtlebot3_gazebo package

    pkg_path = FindPackageShare('april_bot_world') #current package

    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py']) #path to launch file for ros/gazebo sim
    turtlebot3_spawn_launch_path = PathJoinSubstitution([turtlebot3_gazebo_pkg_path, 'launch', 'spawn_turtlebot3.launch.py']) #path to launch file for spawning turtlebot3
    turtlebot3_state_publisher_launch_path = PathJoinSubstitution([turtlebot3_gazebo_pkg_path, 'launch', 'robot_state_publisher.launch.py']) #path to turtlebot3 state publisher launch file

    x_pose = LaunchConfiguration('x_pose', default='-2.0') #set the x position of turtlebot
    y_pose = LaunchConfiguration('y_pose', default='-0.5') #set the y position of turtlebot    

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([pkg_path, 'models'])
        ),
        
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            PathJoinSubstitution([pkg_path, 'models'])
        ),
        
        SetEnvironmentVariable(
            'TURTLEBOT3_MODEL',
            'waffle'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': PathJoinSubstitution([pkg_path, 'worlds/apriltag_world.sdf']),
                'on_exit_shutdown': 'True'
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot3_state_publisher_launch_path),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot3_spawn_launch_path),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose
            }.items(),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_footprint',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
            parameters=[{'use_sim_time': True}]
        ),

        # Bridging and remapping Gazebo topics to ROS 2 (replace with your own topics)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            ],
            remappings=[
                ('/camera/image_raw', '/camera/image_raw'),
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])