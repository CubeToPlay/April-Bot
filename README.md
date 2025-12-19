# April-Bot
## To run the system:
1. Ensure you have all of the necessary packages in the system
2. Use the APRIL-BOT directory as your workspace and build the system in it
3. Run "ros2 launch april_bot_system full_system_launch.py" to run the entire system
4. Give a number gesture (1 - 5) to the web-cam, and the robot will begin either moving towards it (if it has its location in the JSON file) or it will begin exploring the environment searching for it
5. If you want to cancel the search, give the "cancel" gesture and the robot will stop moving
6. If you give any other gesture while the robot is searching, the system will ignore that gesture.
