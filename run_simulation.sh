source /opt/ros/iron/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py

source /opt/ros/iron/setup.bash
source /ros2_ws/install/setup.bash
ros2 run yolo_pkg yolo_subscriber

source /opt/ros/iron/setup.bash
source /ros2_ws/install/setup.bash
ros2 run cow_pkg repeller

source /opt/ros/iron/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch sjtu_drone_bringup bringup.launch.py
