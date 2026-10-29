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
they no longer come from the local grid configuration. The default field is
**40 m × 40 m**, centered at the origin (X and Y from -20 to 20 m). At 0.5 m
resolution this uses 6,400 global coverage cells, down from 40,000. Defaults are a goal at
`(-10, -10)`, a **2 m planar completion radius**, and a conservative rectangular
observation footprint inside a camera sector with a 2–10 m range and 2.09 rad
horizontal field of view.

To run another starting layout, copy this XML, change the positions, and launch:

```bash
ros2 launch sjtu_drone_bringup bringup.launch.py \
  simulation_config_file:=/absolute/path/to/simulation.xml
```

The observation footprint must fit inside the detector's configured range and
field of view. Viewpoints center each search patch in the modeled camera
footprint. The local navigation solver and visualizations remain configurable in
`grid_visualizer_pkg/config/grid_config.xml`. Individual grid nodes also accept
`show_vector_grid:=false`, `show_occupancy_grid:=false`, and
`show_coverage_map:=false` for display-free tests.

Each drone also opens a **Visited Areas** window showing its independent coverage
over the whole field, with green visited cells, white unvisited cells, and a
coverage percentage, plus red dots for current detected/tracked cows. Labeled
markers show the current drone in blue, peers in purple, and the final goal as
an amber crosshair. Drone labels use their configured indices (D0, D1, etc.),
with the current drone marked as self. It includes
both its own observations and received peer observations, without movement
arrows. Cow dots use the detector topic rather than Gazebo ground truth.
Confirmed cows stay at their last detected global positions while out of view;
looking away and leaving the local map do not erase them. Repeated successful
images that inspect a remembered location without detecting its cow invalidate
that entry after `max_lost_frames` (15 by default). Short detection gaps preserve
memory, and a matching detection resets the miss counter. Absence is checked
inside the conservative processed ground footprint, not the wider association
range. Removed entries disappear from maps and navigation/recovery objectives
on the next updates. The view keeps the field
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

## Cow spacing and steering

`cow_pkg/config/cow_config.xml` sets the cow-to-cow repulsion range through
`cow_equilibrium_distance_m`, now **5 m between centers** instead of 4 m.
`cow_repulsion_shape` is reduced from 8 to 3 so repulsion is appreciable before
the 2 m-long cow bodies touch. At 3 m separation, the repulsive magnitude is
0.064 instead of approximately 0.000015. This produces a more spread-out herd;
the 2 m mission completion radius is unchanged.

Both forward speed and turning now taper with resultant force. Resultants at or
below `force_stop_threshold` (0.001) stop both, and `heading_tolerance_rad`
(0.05 rad, about 2.9 degrees) prevents steering corrections for tiny heading
errors. The steering gains are P=1.0 and angular-rate damping=0.2. Damping uses
the cow's measured angular velocity from `/gazebo/model_states`, in rad/s,
instead of yaw changes between messages with an unspecified time interval.
Missing cows and invalid steering inputs receive zero velocity commands.

Rebuild `cow_pkg` and restart the mission to apply these settings. Tests cover
head-on approach with drone pressure and braking lag, bounded steering that
settles near the requested heading, noisy weak forces, measured angular-rate
feedback, and the completion stop latch. The approach test is a controller
simulation; it does not model Gazebo contact forces or guarantee collision-free
motion for every herd layout.

## Drone spacing

Each fellow drone creates a circular obstacle with a **one-cell radius** (0.5 m
at the default 0.5 m resolution), rebuilt from its latest global pose on every
local update. These fixed cells use potential **1.0**, matching cow and map
boundaries. All potential cells remain in **[0, 1]**, including intermediate
relaxation updates. Global search planning
uses the same exclusion radius, and local navigation and recovery avoid the
inflated cells. A drone already inside an exclusion holds until the area clears.

Tune `peer_drone_obstacle_radius_cells` in
`grid_visualizer_pkg/config/grid_config.xml`; rebuild the package and restart
the nodes to apply changes. `peer_drone_repulsion_potential` must equal
`boundary_potential`, and configured potentials must stay in [0, 1].

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
fixed, cell-aligned patches up to 2 m square into bands by distance from the
nearest field edge, with band width equal to half the camera range (5 m by
default). It uses the outermost band with a reachable, non-retired viewpoint.
Once that band has been observed, selection moves inward; unavailable outer
bands are reconsidered on later planning updates.

When selecting a new patch, its viewing position is placed on the line from the
current drone position toward the patch center, offset from that center by the
middle of the modeled camera footprint (about 6.1 m with the defaults). This
puts the attraction between the drone and patch when there is enough viewing
distance. A drone outside an inner patch can therefore look inward instead of
flying past it to a permanently border-facing viewpoint. If already too close,
the camera's minimum viewing distance can require stepping back. Blocked,
retired, or geometrically invalid viewpoints use the nearest valid alternative
angle. Every candidate must fit the entire patch with the 0.5 m margin.

Selection scores obstacle-aware travel distance and peer crowding as
`1 / (1 + travel_distance / camera_range) / crowding_penalty`, with the existing
outward startup preference on the first selection. Each drone retains one
selected attraction and direction until the patch is observed or invalidated;
its viewpoint does not move as the drone approaches or as coverage is partial.
Camera attention compares the distance to this selected attraction against the
distance to the eligible local cow-group center, facing the patch when the
attraction is closer and the cows otherwise.

Set `<search_border_first>0</search_border_first>` in `grid_config.xml` (or the
boolean `search_border_first` ROS parameter at startup) to select nearby
unfinished patches across all bands. Both settings share the same controller.
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

The benchmark now uses the default 40 m × 40 m field. For historical comparison,
on the development machine's 100 m × 100 m scenario, the previous search update
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
  -R 'test_(goal_quiet_zone|search_coverage|stall_recovery|distance_recovery)'
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

## Fixed search patches, approach viewpoints, and initial separation

Search uses fixed, cell-aligned patches up to 2 m by 2 m, reduced automatically
for narrower camera footprints with a 0.5 m geometric margin. A new attraction
is chosen from the drone's current approach side, then its viewing position and
direction remain fixed until the patch is fully observed, blocked, or retired.
Partial observations never shift a patch or its selected attraction. Local and peer camera reports remove
completed patches; reaching a viewpoint alone does not count as coverage.
Outer bands are searched before inner bands, with nearby reachable patches
preferred within a band. Search and cow pushing still share the potential field.

On startup, drones wait for peer poses and initially turn away from the other
drones' centroid before translating. For two drones these headings are opposite.
Coincident starting positions use deterministic headings spaced by drone index.
The first search attraction is biased in that same outward direction; subsequent
patches use normal border-first selection. This discourages duplicate searching
without assigning SEARCH/HERD modes or exclusive territories. Cow forces can
still influence the eventual path. A single drone skips the initial turn.

Camera attention compares two planar distances from the drone: the distance to
the selected global search **attraction/viewpoint**, and the distance to the
arithmetic center of eligible remembered cows in the current local map (20 m
square by default). If the search attraction is strictly closer, the camera faces
the search **patch center**; otherwise it faces the cow-group center. Equal
distances favor the cows. The comparison uses the real attraction rather than
the local grid's clipped intermediate waypoint or the patch center itself.

Cows inside the permanent 2 m goal completion radius are excluded. Local-map
membership is independent of camera heading/range and the temporary 8 m herding
quiet radius. With only one available target type the camera faces it; with
neither it preserves heading. Camera choice is independent of navigation's
60/40 attraction allocation and does not change the selected search attraction.

Search and cow attraction shares are configured in
`ros_ws/src/grid_visualizer_pkg/config/grid_config.xml` using
`search_attraction_percent` (60) and `cow_attraction_percent` (40). Values must
be nonnegative and total 100. This fixed split replaces progress-based balancing;
when only one target type is available, it receives the full budget. Cow points
share their group budget according to their existing priorities and distances.
Rebuild `grid_visualizer_pkg` and restart the nodes after editing the XML.

## Distance-field recovery

Relaxation remains the normal controller, with the configured 60% search / 40%
cow attraction allocation. `enable_geodesic_field_fallback=1` enables recovery
when measured motion stalls or oscillates, or when relaxation fails to converge.
The default stall window is 6 simulation seconds and the confinement threshold
is 0.4 m. Larger back-and-forth motion also qualifies when it contains at least
two reversals, at least 0.8 m of travel, and net displacement below 25% of travel.
Motion segments below 0.05 m do not count as reversals. Intentional holds within
0.5 m of a search viewpoint do not trigger recovery. Cow-pushing waypoints are
not terminal holds: the normal field remains active there, and a stalled cow
approach remains eligible for recovery.

Recovery selects one reachable target using obstacle-aware distance plus its
existing attraction-priority cost. It retains the global target while rebuilding
a single-target distance field around moving obstacles and the moving local
window. Search targets retain their patch identity; cow targets use spatial
association within the existing 2 m association radius because the cow topic
contains positions without tracker IDs. Invalid or unreachable targets are
replaced. Zero-strength targets cannot seed recovery. With no safe route, the
drone explicitly holds rather than following a stale waypoint.

`recovery_release_m` now means **minimum approach toward the retained target**
(default 1 m), rather than displacement from the stall position. Returning to
relaxation additionally requires a converged field, normal and recovery headings
within `recovery_alignment_deg` (45 degrees) for `recovery_agreement_s` (3 s),
and at least `recovery_window_progress_m` (0.3 m) of approach in that window.
Target motion beyond one grid cell resets the progress reference. Arrival within
`recovery_arrival_m` (0.5 m) also ends recovery; reaching a clipped intermediate
waypoint does not count as arrival.

After handoff, `recovery_watch_s` (6 s) retains the target. Losing
`recovery_backtrack_m` (1 m) from the best distance, or opposing the recovery
route for `recovery_opposition_s` (1 s), immediately reactivates recovery.
Simulation clock resets, mission stop, and loss of objectives clear recovery
memory. This adds navigation recovery memory, not SEARCH/HERD task modes.

Recovery respects fixed cells, prevents diagonal corner cutting, and caps each
step at the smaller of one grid cell and `max_goto_movement_m`. The vector view
keeps the relaxed field arrows and adds a magenta commanded arrow and retained
target marker, labelled RECOVERY or HANDOFF WATCH. Transition logs identify the
trigger and selected target. Camera attention keeps using the search-attraction
versus cow-center distance comparison during navigation recovery.

Thresholds are in `ros_ws/src/grid_visualizer_pkg/config/grid_config.xml`.
Rebuild `grid_visualizer_pkg` and restart the nodes to apply them. Recovery still
uses a local map and does not guarantee global completeness in every obstacle
layout. The paper's adaptive potential distortion is not implemented.

Standalone checks include a closed-loop saddle fixture using the configured
controller's velocity, acceleration, slowdown, and position tolerance. Run the
live headless recovery check after building and sourcing the workspace, in an
isolated ROS domain:

```bash
ROS_DOMAIN_ID=157 python3 -m pytest -q \
  src/grid_visualizer_pkg/test/test_peer_avoidance_ros.py \
  src/grid_visualizer_pkg/test/test_distance_recovery_ros.py
```

The ROS check covers measured stalls, retained targets, stable-progress handoff,
command bounds, camera priority, simulation-clock resets, and mission stop.
A full Gazebo reproduction of the reported camera/detection scene remains a
separate validation step.

## Experiment batteries and replay

Run `./scripts/test_lab.sh` from the repository root and open
**http://127.0.0.1:8765** to create starting layouts, queue repeated simulations,
and replay actual cow/drone trajectories with fading paths. Runs end at success
or at **20 simulated minutes** and save per-entity movement metrics, both clocks,
configuration snapshots and logs locally under `ros_ws/test_results/`. The
detector's remembered cows are available as a separate replay overlay. New tests
also record per-drone searched/unsearched maps and search goals. Replay has layer
toggles, scenario/live-run controls for desktop viewers, and single/bulk
experiment deletion.

See [Herding Lab setup, CLI batches and data format](ros_ws/src/simulation_test_pkg/README.md).
