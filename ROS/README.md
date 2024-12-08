## Adding files to ROS project on the Jetson
1. Add new ROS nodes to ~/kraken/ROS/src/kraken/kraken
2. Add included files to ~kraken/ROS/src/kraken/kraken/include
3. Open ~/kraken/ROS/src/kraken/setup.py
4. Within "console_scripts", add "<node_name> = kraken.<file_name>:main"
5. Run "colcon build --packages-select kraken"

## Running a node on the jetson
1. Run "source ~/kraken/ROS/install/setup.bash"
2. Run "cd ~/kraken/ROS/src"
3. Run "ros2 run kraken <node_name>"
