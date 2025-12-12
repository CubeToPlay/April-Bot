from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim') #ros/gazebo package
    turtlebot3_gazebo_pkg_path = get_package_share_directory('turtlebot3_gazebo') #turtlebot3_gazebo package

    pkg_share = get_package_share_directory('april_bot_world')
    pkg_path = FindPackageShare('april_bot_world') #current package

    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py']) #path to launch file for ros/gazebo sim
    turtlebot3_spawn_launch_path = PathJoinSubstitution([turtlebot3_gazebo_pkg_path, 'launch', 'spawn_turtlebot3.launch.py']) #path to launch file for spawning turtlebot3
    turtlebot3_state_publisher_launch_path = PathJoinSubstitution([turtlebot3_gazebo_pkg_path, 'launch', 'robot_state_publisher.launch.py']) #path to turtlebot3 state publisher launch file

    x_pose = LaunchConfiguration('x_pose', default='-2.0') #set the x position of turtlebot
    y_pose = LaunchConfiguration('y_pose', default='-0.5') #set the y position of turtlebot    
    models_path = os.path.join(pkg_share, 'models')
    world_path = os.path.join(pkg_share, 'worlds', 'apriltag_world.sdf')
    def write_fastdds_file(context):
        xml_content = """<?xml version="1.0" encoding="UTF-8"?>
<dds>
  <profiles>

    <transport_descriptors>
      <transport_descriptor>
        <transport_id>udp_only</transport_id>
        <type>UDPv4</type>
      </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="udp_profile" is_default_profile="true">
      <rtps>
        <use_builtin_transports>false</use_builtin_transports>
        <user_transports>
          <transport_id>udp_only</transport_id>
        </user_transports>
      </rtps>
    </participant>

  </profiles>
</dds>
    """
        path = os.path.expanduser("~/.ros/fastdds.xml")
        with open(path, 'w') as f:
            f.write(xml_content)
        return []
    return LaunchDescription([
        OpaqueFunction(function=write_fastdds_file),

        SetEnvironmentVariable(
            name="FASTDDS_DEFAULT_PROFILES_FILE",
            value=os.path.expanduser("~/.ros/fastdds.xml")
        ),

        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([pkg_path, 'models'])
        ),
        
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            PathJoinSubstitution([pkg_path, 'models'])
        ),
        
        # SetEnvironmentVariable(
        #     'TURTLEBOT3_MODEL',
        #     'waffle'
        # ),
        # Launch Gazebo with your world
        ExecuteProcess(
            cmd=['gz', 'sim', world_path, '-r'],
            output='screen',
            shell=False
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(gz_launch_path),
        #     launch_arguments={
        #         'gz_args': PathJoinSubstitution([pkg_path, 'worlds/apriltag_world.sdf']),
        #         'on_exit_shutdown': 'True'
        #     }.items(),
        # ),
        # TimerAction(
        #     period=3.0,
        #     actions=[
        #         Node(
        #             package='ros_gz_sim',
        #             executable='create',
        #             name='spawn_turtlebot3',
        #             arguments=[
        #                 '-world', 'apriltag_world',
        #                 '-name', 'turtlebot3_waffle', 
        #                 '-file', os.path.join(models_path, 'turtlebot3_waffle_gz', 'model.sdf'),
        #                 '-x', '-2.0',
        #                 '-y', '-0.5',
        #                 '-z', '0.01',
        #             ],
        #             output='screen',
        #             parameters=[{'use_sim_time': True}]
        #         ),
        #     ]
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(gz_launch_path),
        #     launch_arguments={
        #         'gz_args': PathJoinSubstitution([pkg_path, 'worlds/apriltag_world.sdf']),
        #         'on_exit_shutdown': 'True'
        #     }.items(),
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(turtlebot3_state_publisher_launch_path),
        # ),
        #  Node(
        #     package='ros_gz_sim',
        #     executable='spawn_entity.py',
        #     arguments=[
        #         '-topic', 'robot_description',
        #         '-entity', 'waffle',
        #         '-x', x_pose,
        #         '-y', y_pose,
        #         '-z', '0.01',
        #     ],
        #     output='screen',
        #     parameters=[{'use_sim_time': True}]
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(turtlebot3_spawn_launch_path),
        #     launch_arguments={
        #         'x_pose': x_pose,
        #         'y_pose': y_pose
        #     }.items(),
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='odom_to_base_footprint',
        #     output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        #     parameters=[{'use_sim_time': True}]
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_footprint_to_base_scan',
        #     output='screen',
        #     arguments=['0', '0', '0.2', '0', '0', '0', 'base_footprint', 'base_scan'],
        #     parameters=[{'use_sim_time': True}]
        # ),
        # Static transform: odom -> base_footprint (backup)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_footprint_backup',
            output='screen',
            arguments=['--frame-id', 'odom', '--child-frame-id', 'base_footprint'],
            parameters=[{'use_sim_time': True}]
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
        # Bridging and remapping Gazebo topics to ROS 2 (replace with your own topics)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
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
            parameters=[{
                'use_sim_time': True,
                'qos_overrides./scan.reliability': 'best_effort',
            }],
            output='screen'
        ),
    ])