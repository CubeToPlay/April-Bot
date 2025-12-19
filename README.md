# April-Bot

## Dependencies
<!-- base packages -->
### Base ROS2 Packages
ROS2 base and slam packages
turtlebot3_gazebo, rvis, rqt, slam_toolbox, cv_bridge, etc.

<!-- mediapipe -->
<!-- This is what I used for my project. If you can figure a way to install it without --break-system-packages, go ahead. -->
### Python Packages
mediapipe
`pip install mediapipe --break-system-packages`

## To run the system:
1. Ensure you have all of the necessary packages in the system
2. Use the `April-Bot` directory as your workspace
3. Build the system using `colcon build --packages-up-to april_bot_system` or just `colcon build`
4. Run `ros2 launch april_bot_system full_system_launch.py` to run the entire system. Smaller subsystems can be ran using their own launch files
5. Give an ASL number gesture (1 - 5) to the webcam and hold it for 3 seconds, and the robot will begin either moving towards it (if it has its location in the JSON file) or it will begin exploring the environment searching for it. Give it some time. If you look at the terminal logs, it should say something like `Gesture 3 chosen`
6. If you want to cancel the search, give the "cancel" gesture and the robot will stop moving. Thumb and pointer extended
7. If you give any other gesture while the robot is searching, the system will ignore that gesture.


## Additional Information
The `submission` folder has the video and slides.
