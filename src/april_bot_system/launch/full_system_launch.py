from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#writing-launch-files
def generate_launch_description():
    april_bot_world_launch_dir = PathJoinSubstitution([FindPackageShare('april_bot_world'), 'launch'])
    goal_launch_dir = PathJoinSubstitution([FindPackageShare('goal'), 'launch'])
    hand_gestures_launch_dir = PathJoinSubstitution([FindPackageShare('hand_gestures'), 'launch'])
    april_tag_navigator_launch_dir = PathJoinSubstitution([FindPackageShare('april_tag_navigator'), 'launch'])
    april_tag_detector_launch_dir = PathJoinSubstitution([FindPackageShare('april_tag_detector'), 'launch'])
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([april_bot_world_launch_dir, 'april_world_turtlebot3.launch.py'])
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([goal_launch_dir, 'goal_launch.py'])
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([hand_gestures_launch_dir, 'hand_gestures_launch.py'])
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([april_tag_navigator_launch_dir, 'navigator.launch.py'])
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([april_tag_detector_launch_dir, 'detector.launch.py'])
        ),
    ])