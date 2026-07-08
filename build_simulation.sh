sudo apt update
sudo apt install ros-iron-rviz2

cd /ros2_ws/
rosdep update --rosdistro=iron
source /opt/ros/iron/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build

colcon build --packages-select cow_pkg

docker cp ./ros_ws/models/. drone_herding:/root/.gazebo/models/