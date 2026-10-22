The simulation selects its cow tracker through `config/yolo_config.xml`:

```xml
<tracker use_kalman_filter="false" ... />
```

- `false` (default): `cow_tracker.py` associates detections by distance and
  stores the latest measured position directly. It has no Kalman state,
  smoothing, velocity estimation, or motion prediction.
- `true`: `cow_tracker_with_kalman_filter.py` retains the Kalman filter and
  prediction used to associate and smooth observed cow positions.

Both versions confirm new cows using `min_confirmation_hits` and then retain
them for the entire running mission. Missing detections, looking away, and
leaving a drone's local map do not delete a confirmed cow or move its last
mapped global position. Reacquisition updates that same entry when association
succeeds. `max_lost_frames` (15 by default) expires only **unconfirmed candidates**
after visible misses; out-of-view updates do not advance that counter.
Camera synchronization and merging of overlapping
camera detections remain common to both versions; unfiltered positions are
the measurements produced by that existing detection pipeline.

Visibility still uses the horizontal FOV and range approximation. Switching
trackers does not correct its known vertical-view, camera-attitude, or
occlusion limitations.

Build inside the simulation container, then launch the simulation normally.
An already running YOLO node must be restarted to load a changed selection.
The startup log reports the selected tracker module.

```bash
cd /ros2_ws
source /opt/ros/iron/setup.bash
colcon build --packages-select yolo_pkg
source install/setup.bash
```

Run the tracker regression tests without starting Gazebo:

```bash
PYTHONPATH=src/yolo_pkg python3 -m unittest discover \
  -s src/yolo_pkg/test -p test_cow_tracker.py -v
```
