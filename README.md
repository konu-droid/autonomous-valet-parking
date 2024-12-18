This is KVS Mohan Vamsi's mobile robotics project on autonomous valet parking.

Please setup ros2 humble enironment before running the code. 

All the simulation environments can be found in USD folder. Load and start any one enivronment to begin. For example: parking_env.usd

once the isaac sim simulation is start. run ```ros2 launch intensity_slam slam_launch.py``` This will start the rviz and mapping node.

Use ```ros2 run turtlebot3_teleop teleop_keyboard``` to move the robot around in the env

Once the map ismade close the node to save the map.

Please run ```ros2 run turtlebot3_teleop teleop_keyboard``` to run the nav2 stack and run the robot around in the map.

use ```ros2 run intensity_slam GUI``` to start the gui for parking
