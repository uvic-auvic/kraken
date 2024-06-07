# kraken
Software repository for AUVIC's kraken AUV.

## Adding files to ROS project on the Jetson
1. Add new ROS nodes to ~/kraken/src/kraken/kraken
2. Add included files to ~kraken/src/kraken/kraken/include
3. Open ~/kraken/src/kraken/setup.py
4. Within "console_scripts", add "<node_name> = kraken.<file_name>:main"
5. Run "colcon build --packages-select kraken"
6. Run "source install/setup.bash"

## Running a node on the jetson
1. Run "cd ~/kraken/src"
2. Run "ros2 run kraken <node_name>"
