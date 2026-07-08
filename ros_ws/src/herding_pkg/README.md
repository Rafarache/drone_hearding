cd /ros2_ws
colcon build --packages-select yolo_pkg
source install/setup.bash

source /opt/ros/iron/setup.bash
source /ros2_ws/install/setup.bash
ros2 run yolo_pkg yolo_subscriber