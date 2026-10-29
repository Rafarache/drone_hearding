The detector publishes raw per-camera observations with **cow tracking disabled**.
Each processed image immediately publishes `/cow_observations` with the image
timestamp, source drone index, interpolated camera pose, cone limits, global
positions, position-valid flags, and bounding-box angular spans. Valid empty
frames are published; missing pose or failed image processing is not empty evidence.

`/cows_pos` remains a raw `PoseArray` for compatibility. It contains only valid
positions from that camera frame, with the source index in `position.z`. Empty
frames contain no poses. There is no batching across cameras, confirmation,
association, prediction, or republishing of remembered cows.

`config/yolo_config.xml` configures detection, projection, and camera display.
Tracker and tracker-plot settings are removed from the active configuration.
Historical tracker modules and their standalone tests remain unused.

Navigation consumes `/cow_observations`, not `/cows_pos`. Each drone maintains
cell confidence from 0 to 255: own hits +150, own visible empty space −30, then
peer hits +150. Cells above 180 represent cows. Any hit in the current map update
protects its cell from subtraction. Cow bounding boxes also protect occluded
space in the own camera view. See the repository README for visibility and
multi-drone update semantics. The existing empirical position calibration is unchanged.

Build the interfaces and consumers together, then restart the simulation nodes:

```bash
cd /ros2_ws
source /opt/ros/iron/setup.bash
colcon build --packages-select cow_mapping_interfaces yolo_pkg grid_visualizer_pkg sjtu_drone_bringup
source install/setup.bash
```

Test deterministic camera callbacks without Gazebo or model downloads:

```bash
ROS_DOMAIN_ID=162 PYTHONPATH=src/yolo_pkg python3 -m pytest -q \
  src/yolo_pkg/test/test_raw_observations.py \
  src/yolo_pkg/test/test_search_observation.py
```
