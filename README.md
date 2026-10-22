# Distributed search and cow herding

The canonical launch runs cow detection, an independent search/navigation node
for each drone, position controllers, cow movement, and a completion monitor:

```bash
cd ros_ws
source /opt/ros/iron/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
ros2 launch sjtu_drone_bringup bringup.launch.py
```

The supplied Docker environment uses `/ros2_ws` as the workspace. As before, the
normal launch needs a display for RViz, camera/grid windows, and keyboard
terminals, plus the Gazebo models and YOLO weights used by this repository.

## Configuration

`ros_ws/src/sjtu_drone_bringup/config/simulation.xml` defines the field bounds,
drone and cow starting positions, goal, completion radius, and search footprint.
The goal and radius are passed to both navigation and completion monitoring;
they no longer come from the local grid configuration. Defaults are a goal at
`(-10, -10)`, a **2 m planar completion radius**, and a conservative rectangular
observation footprint inside a camera sector with a 2–10 m range and 2.09 rad
horizontal field of view.

To run another starting layout, copy this XML, change the positions, and launch:

```bash
ros2 launch sjtu_drone_bringup bringup.launch.py \
  simulation_config_file:=/absolute/path/to/simulation.xml
```

The observation footprint must fit inside the detector's configured range and
field of view. The 5 m search viewpoint offset must lie between its near/far
ranges. The local navigation solver and visualizations remain configurable in
`grid_visualizer_pkg/config/grid_config.xml`. Individual grid nodes also accept
`show_vector_grid:=false`, `show_occupancy_grid:=false`, and
`show_coverage_map:=false` for display-free tests.

Each drone also opens a **Visited Areas** window showing its independent coverage
over the whole field, with green visited cells, white unvisited cells, and a
coverage percentage, plus red dots for current detected/tracked cows. It includes
both its own observations and received peer observations, without movement
arrows. Cow dots use the detector topic rather than Gazebo ground truth.
Confirmed cows stay at their last detected global positions until reacquired;
looking away, missed detections, and leaving the local map cannot delete them.
The tracker retains this memory until the mission processes restart. The view keeps the field
bounds fixed and remains responsive after the mission stops.

For off-map cows, local arrow/occupancy views show a hollow red edge marker
labeled with the cow's distance. The controller keeps the cow's global position
separate from its clipped local push target (green), recalculating the target as
the drone moves. Clipping never overwrites the remembered cow position.

Set `<show_coverage_map>1</show_coverage_map>` in `grid_config.xml` to enable these
windows by default (use `0` to disable them), then rebuild the package. The boolean
ROS parameter can also toggle an individual drone's window while it is running:

```bash
ros2 param set /simple_drone0/grid0 show_coverage_map false
ros2 param set /simple_drone0/grid0 show_coverage_map true
```

## Independent coverage maps

Each drone's grid node owns a permanent 0.5 m coverage grid spanning the field.
A successfully processed camera image with a matched image-time pose creates an
observation, even if it contains no detections. Observations are sampled at most
1 Hz per camera before updating any coverage map. Set `observation_period_s`
on `<search>` in `simulation.xml` to change the interval (for example, `2.0`
sends at most once every two seconds). The detector's corresponding startup ROS
parameter is `search_observation_period_s`. Frames that cannot be
processed or matched to a pose do not count as searched.

Each message carries just four corners of a box rotated with the camera heading,
instead of the previous 50-point cone outline. The box is inscribed in the
camera sector, so it conservatively leaves some visible ground unmarked rather
than marking unseen ground. Its forward limits are approximately 4–8.14 m and
its width is 11.61 m with the defaults. Both search gain scoring and transmitted
observations use this same shape. Local and receiving drones rasterize the same
box into their independent grids; inference and cow detections keep their own
cadence. This reduces coordinate payload from 3,000 to 48 bytes/second per
camera at the defaults (excluding ROS/DDS headers).

| Topic | Message | Purpose |
| --- | --- | --- |
| `/simple_droneN/processed_view` | `geometry_msgs/PolygonStamped` | Detector's image-time footprint in `map` |
| `/simple_droneN/searched_area` | `geometry_msgs/PolygonStamped` | Drone N's own inspected footprint, broadcast once |
| `/simple_droneN/goto` | `geometry_msgs/Pose` | Current navigation waypoint or explicit hold |
| `/simple_droneN/focusin` | `geometry_msgs/Pose` | Camera/yaw attention target |
| `/simulation/stop` | `std_msgs/Empty` | Permanent mission stop |

The namespace prefix is configurable. Coverage topics are reliable and volatile:
peers rasterize incoming polygons into **their own** grids and never rebroadcast
them. There is no central coverage map, replay, resynchronization, or aging.
A restarted or disconnected drone therefore can have less coverage knowledge.
Navigation waits for its first valid own-camera observation before moving.

Coverage means that an area was inside the modeled camera footprint. It is not
proof that a cow was detected. Static occlusion and obstacle mapping are outside
this implementation; the existing cow, peer-drone, and field exclusions apply.

## Simultaneous objectives

By default, MMPF-inspired selection searches **from the borders inward**. It groups
unseen cells into bands by distance from the nearest field edge, with band width
equal to half the camera range (5 m by default), and splits each band into
camera-range tiles. It uses the outermost band with a reachable, non-retired,
positive-gain viewpoint. Once that band has been covered, selection moves to the
next band inward; temporarily unavailable outer bands are reconsidered every
planning update. This lets a drone starting in the center select distant border targets
instead of growing the local searched frontier.

For each tile, selection takes a representative unseen cell near its centroid,
samples eight viewpoints 5 m around it, and scores predicted new coverage of the
active band against wavefront travel distance and nearby peer crowding. The score
is `new_band_cells / (1 + travel_distance / camera_range) / crowding_penalty`.
The best viewpoint per tile is retained, and up to 12 scored viewpoints feed the
field. Camera search attention also scores the active band so central unknown
space cannot steal attention from a border observation. Search targets are
observation positions inside the field; the drone need not fly on the fence.

Set `<search_border_first>0</search_border_first>` in `grid_config.xml` (or the
boolean `search_border_first` ROS parameter at startup) to use the original
inside-out selector, which clusters unseen cells bordering already searched
cells. Both policies feed the same simultaneous search/pushing controller.
Wavefront travel rejects exclusions and diagonal corner cutting. A viewpoint
that produces no new coverage in three aligned arrival observations is retired
for 10 simulation seconds so an alternative can be selected.

Global search destination selection runs at most once per second by default
(`search_planning_period_s`, a positive startup parameter on each grid node).
The local field still updates at 5 Hz and movement commands at 2 Hz. Cached search
destinations that become blocked, covered, or retired are removed on the next
local update. The whole-map display refreshes at 1 Hz, caches unchanged coverage
cells, and continues drawing the latest remembered cow positions in red. Its
runtime enable/disable boolean still applies immediately on the next UI tick.
Ordinary CMake builds now default to `RelWithDebInfo` for this C++ package;
explicit build types are respected and regression assertions remain enabled.

To profile search separately from Gazebo, from the repository root:

```bash
g++ -std=c++17 -O2 -Iros_ws/src/grid_visualizer_pkg/include \
  ros_ws/src/grid_visualizer_pkg/test/benchmark_search_coverage.cpp -o /tmp/benchmark_search
/tmp/benchmark_search
```

On the development machine's 100 m × 100 m scenario, the previous search update
took about 65 ms without optimization (the existing build default), or 13 ms
with `-O2`. The box-based version takes about 6.3 ms with `-O2` and runs at 1 Hz
instead of 5 Hz. Forty camera gain evaluations fell from 2.6 ms to 0.22 ms when
both versions used `-O2`; each heading is now evaluated once instead of twice.
These are isolated CPU timings, not a measurement of full simulation speed.

Search viewpoints and existing cow push points feed **one scalar field every
cycle**. There are no search/herding phases. Each active group starts with equal
weight; scores are normalized within groups. Lack of coverage or cow-distance
progress increases that group's weight by a bounded multiplier from 1 to 2 over
30 simulation seconds. Cow progress uses geometric association and two 5-second
median distance windows. When only one group has objectives, it receives the
full weight.

Objective cells are weighted absorbing cells, with the discrete update
`U = (left + right + up + down) / (4 + lambda)`. Other free cells use Laplace's
equation, while exclusion and field boundaries retain their high fixed values.
The absorption budget is 32 at 0.5 m resolution, scaled by cell area; coincident
objectives accumulate with a cap. This allows strengths to affect the same
solve without making weak targets repulsive. Existing gradient navigation and
geodesic recovery use this field. Invalid/nonconverged fields publish a hold,
replacing stale waypoints. Camera heading scores both new coverage and visible
active cows with the same group weights.

This is a heuristic balance, not a guarantee of complete coverage or finite-time
herding for every layout. Peer crowding reduces duplicate effort without a
central destination allocator. Coverage remains permanent even though cows can
move into previously inspected regions.

## Completion and stopping

`cow_pkg/simulation_completion_monitor` subscribes to `/gazebo/model_states`.
It requires every configured cow, with a unique name and finite position, to be
inside the goal radius in the **same snapshot**. There is no dwell time or
coverage-completion requirement, and detector tracks are not used for this
decision.

The monitor publishes a reliable, transient-local stop and stays alive so late
subscribers receive it. Cow and drone movement controllers latch it permanently,
clear queued targets, and repeat zero velocity commands. The flight plugin also
rejects subsequent commands, mode changes, and resets; it cancels takeoff/landing
and bypasses command noise. Airborne drones brake through normal physics and
hover, so stopping is not an instantaneous pose freeze. Gazebo and visualization
windows remain open. Restart the mission processes to begin another run.

## Validation

After building and sourcing the workspace:

```bash
ctest --test-dir build/grid_visualizer_pkg --output-on-failure \
  -R 'test_(goal_quiet_zone|search_coverage)'
python3 -m pytest -q \
  src/cow_pkg/test/test_completion.py \
  src/cow_pkg/test/test_repeller_forces.py \
  src/cow_pkg/test/test_interaction.py \
  src/cow_pkg/test/test_motion.py \
  src/yolo_pkg/test/test_search_observation.py \
  src/yolo_pkg/test/test_cow_tracker.py \
  src/sjtu_drone_bringup/test/test_search_mission.py \
  src/sjtu_drone_bringup/test/test_gazebo_stop.py
```

Run ROS integration tests in an isolated ROS domain with no active mission.
They cover live coverage exchange, simultaneous objectives, empty cow tracks,
completion edge cases, controller latching, late subscribers, and the real
Gazebo flight plugin during takeoff, landing, and position control. They do not
replace an end-to-end camera/detection mission trial with cows distributed
across the full field.
