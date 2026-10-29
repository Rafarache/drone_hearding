The simulation selects its cow tracker through `config/yolo_config.xml`:

```xml
<tracker use_kalman_filter="false" ... />
```

- `false` (default): `cow_tracker.py` associates detections by distance and
  stores the latest measured position directly. It has no Kalman state,
  smoothing, velocity estimation, or motion prediction.
- `true`: `cow_tracker_with_kalman_filter.py` retains the Kalman filter and
  prediction used to associate and smooth observed cow positions.

Both versions confirm new cows using `min_confirmation_hits` and retain their
last mapped global positions while out of view. A short detection gap does not
move or erase a cow, and reacquisition resets its miss counter. After
`max_lost_frames` (15 by default) processed empty observations of its remembered
location, any track, including a confirmed cow, is removed. Looking away,
leaving a drone's local map, missing pose matches, and failed image processing
supply no negative evidence and cannot expire a track.

In the detector, negative evidence uses the conservative ground rectangle of
each successfully processed image, independent of the 1 Hz coverage-publication
throttle. The wider FOV/range remains available for matching detections but does
not by itself establish that an absent cow's ground location was inspected.
An empty tracked `PoseArray` is published when the last track is removed, so
navigation and retained recovery targets can clear immediately.
Camera synchronization and merging of overlapping
camera detections remain common to both versions; unfiltered positions are
the measurements produced by that existing detection pipeline.

The ground footprint is still a camera model; camera-attitude errors and static
occlusion are not modeled. Switching trackers does not correct those limitations.

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
