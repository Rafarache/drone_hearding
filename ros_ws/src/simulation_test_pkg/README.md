# Herding Lab

A local test battery runner and trajectory replay application for the canonical
`ros2 launch sjtu_drone_bringup bringup.launch.py` mission.

## Start the application

From the repository root, with Docker and Node.js 20.19+ or 22.12+ available:

```bash
./scripts/test_lab.sh
```

Open **http://127.0.0.1:8765**. The script uses the existing `drone_herding`
container, installs isolated Python dependencies and virtual-display support,
builds the affected ROS packages and React frontend, and starts the local API.
It expects the existing host-network container with `ros_ws` mounted at
`/ros2_ws`. Set `SIM_TEST_CONTAINER` or `SIM_TEST_PORT` to override the defaults.
Use `./scripts/test_lab.sh setup` to prepare dependencies without starting it.
The container must be able to install packages on first setup; subsequent starts
reuse the installed dependencies. No cloud service or account is used.

Create or clone a scenario, edit its map, goal, drone/cow positions and speed
limits, then use **New batch** to request repeats. A batch can contain several
scenarios, each with its own repeat count. Tests run sequentially in fresh worlds.
The dashboard can be closed while the local server keeps running.

**The 20-minute limit uses simulation time**, starting at the first valid clock
and complete configured model snapshot. Real elapsed time and startup time are
recorded separately. Success ends a test early when all cows are simultaneously
inside the completion radius. Slow simulations may take more than 20 real minutes.
Startup fails after 120 real seconds; a clock or expected-model telemetry outage
fails after 60 real seconds. Errors retain partial trajectories and logs, and
the queue continues. Cancelling one test continues the queue; cancelling a batch
cancels its active and remaining tests. Stopping the server stops the active test
and retains the pending queue. Restarting does not silently rerun interrupted
experiments.

## Replay and comparison

Select any run to play, pause, scrub backward or forward, or choose 0.25–8×
playback speed. Paths fade over the selected 15/30/60/120-second simulation-time
window; **Full path** shows the traversed path up to the playhead. Actual cows and
drones have persistent model-name labels. The optional detected-cow overlay shows
the detector's anonymous remembered positions, along with the snapshot age. It
is deliberately separate from actual movement and does not affect metrics.

The Overview table reports each entity's XY and 3D distances, average speeds,
peak planar speed, and cow goal arrival/final distance. Average speed includes
stationary time. Position gaps longer than one simulation second are not bridged;
averages then use the remaining observed interval and are flagged incomplete.
XY is the primary herding metric; 3D includes takeoff and altitude changes.
Distances are sample-based polyline estimates, not exact continuous paths.

Select run checkboxes and **Compare** to inspect results and saved speed limits.
Each run exposes metrics/trajectory CSV downloads, configuration snapshots,
provenance and full logs. **Rerun** reuses that run's frozen configuration files;
cloning its settings into a scenario uses current defaults for other settings.

## Scenario goal radii

The scenario editor exposes three independent radii, all measured from the goal
center in planar meters:

| Setting | Default | Behavior |
| --- | --- | --- |
| Goal radius | 3 m | Cows inside stop generating pushing goals. A new arrival activates the extension. |
| Completion radius | 5 m | All cows inside simultaneously completes the test; metrics use this radius too. |
| Extended goal radius | 5 m | No-push radius for 20 seconds after a new arrival inside the normal goal. |

The extended radius must exceed the normal goal radius. Replay displays the
normal goal circle and a dashed completion circle; a different extended radius
gets a dotted circle. These rings show configured limits. Radius values are
saved in each run's scenario and immutable XML configuration.

JSON/CLI scenarios use `goal.radius_m`, `goal.completion_radius_m` and
`goal.extended_radius_m`. For example:

```json
"goal": {"x": -10, "y": -10, "radius_m": 3, "completion_radius_m": 5, "extended_radius_m": 5}
```

Older scenarios with only `radius_m` retain that shared navigation/completion
radius. Historical results keep their original completion definition, and reruns
recover the extended radius from the frozen grid configuration. New scenarios
use the current defaults shown above; existing saved scenarios can be edited.

## Terminal batteries

A version 1 JSON batch accepts saved scenario IDs or complete inline scenarios:

```json
{
  "version": 1,
  "name": "Five repeats of baseline",
  "items": [{"scenario_id": "COPY_ID_FROM_API", "repeats": 5}]
}
```

Inspect `GET /api/scenarios`, or `GET /api/defaults` for the full scenario shape.
For an inline scenario replace `scenario_id` with `scenario: { ... }`. The same
schema and validation are used by the website and CLI. Each scenario includes
`name`, `description`, `map`, `goal`, `drones`, `cows`, `speeds`, and `duration_s`.
Coordinates and speed values must be finite, all starting positions and the goal
must be within the map, speeds must be positive, and `duration_s` cannot exceed
1,200. Cow starts include `z` (default 0.01 m). Speeds apply to every entity of
that type in the run.

```bash
./scripts/test_lab.sh run /path/to/battery.json
```

Or inside the prepared container/workspace:

```bash
source /opt/ros/iron/setup.bash
source /ros2_ws/install/setup.bash
cd /ros2_ws
.test_lab_venv/bin/python -m simulation_test_pkg.cli --spec battery.json
```

Only one supervisor can own a results directory. Stop the website server before
using CLI execution with the same directory, or supply a different `--data-dir`.
CLI exit code 0 means all experiments produced results (including timeouts);
1 reports infrastructure failure/interruption and 130 reports operator interruption.
The HTTP API offers scenario CRUD, batch creation/cancellation, run reruns,
windowed trajectories, summaries, configuration and downloads. Interactive API
documentation is available at `/docs`.

## Saved data and reproducibility

All recordings live in **`ros_ws/test_results/`**, mounted on the host and ignored
by Git. Keep this directory to preserve history across container restarts.

- `catalog.sqlite3`: saved scenarios, batches and queued/final run metadata.
- `runs/<id>/trajectory.sqlite3`: actual positions at 10 Hz simulation time,
  detector snapshots, per-drone search history, and lifecycle events.
- `runs/<id>/config/`: immutable simulation XML, cow/drone motion settings,
  navigation/detector settings, vehicle YAML and world file used by that run.
  `world.original.world` preserves the source; `world.world` removes archival
  Gazebo `<state>` data so each test starts a fresh clock and model state.
- `manifest.json`, `runtime.json`, `summary.json`, `state.json`: configuration
  hashes, detector weight checksum, entity mapping, isolated ROS/Gazebo settings,
  final metrics, and lifecycle timestamps.
- `launch.log`, `worker.log`, `ros_logs/`: per-run diagnostics.

Actual positions come from `/gazebo/model_states`, matched by model name.
Gazebo's `ModelStates` has no header; samples use the latest received `/clock`
value. Detector snapshots come from the configured cow-position topic.
Telemetry records XYZ, orientation and measured linear velocity; model order in
Gazebo messages does not determine identity. Replay interpolates neighboring
samples, without interpolating across long recording gaps.

The host startup script saves Git revision, working-tree patch and untracked
file names because the container mounts the workspace, not the repository's
`.git` directory. Configuration snapshots are fixed at queue submission. Code,
models and detector weights are not copied into each run; retain the recorded
revision/assets for exact reproduction, and avoid rebuilding code mid-batch.
Repeated runs are comparable experiments, not a promise of bitwise determinism.

Each run uses its own ROS domain, Gazebo master port, stop signal and process
lease. A manual simulation in another domain/master is not stopped. The normal
interactive launch remains available. Automated tests disable manual control
nodes and default to no desktop windows; the optional live-window settings below
enable individual viewers. Cameras continue rendering, using a virtual display
if `DISPLAY` is absent. No camera videos are stored.

## Validation

```bash
# In the ROS container, after sourcing the workspace:
.test_lab_venv/bin/python -m pytest -q src/simulation_test_pkg/test

# On the host:
npm test --prefix ros_ws/src/simulation_test_pkg/web
npm run build --prefix ros_ws/src/simulation_test_pkg/web
# With the local server running and Google Chrome available:
npm run test:browser --prefix ros_ws/src/simulation_test_pkg/web
```

Set `CHROME_BIN` for a different Chromium executable or `LAB_URL` for a different
local port. Browser checks edit and delete a temporary scenario, inspect real
recorded replay data when available, and save desktop/mobile screenshots in
`/tmp`. They do not start a full-length batch. Historical runs made before this
recorder cannot be reconstructed from ordinary ROS text logs.

To repeat the real launch smoke test (one early success and one 20-second
simulation-time run), inside the prepared ROS container:

```bash
SIM_TEST_RUN_REAL=1 .test_lab_venv/bin/python -m pytest -q \
  src/simulation_test_pkg/test/test_runner_ros.py
```

These opt-in tests launch actual Gazebo and YOLO processes; the ordinary unit
suite skips them. Normal interactive simulation launch defaults are preserved.

## Search-map timeline

New tests also record each drone's **actual search memory and selected search
goal**. Expand **Search map & goals** in replay to toggle searched areas,
unsearched areas and search goals independently. Select individual drones to
inspect their knowledge; selecting several shows the union of their recorded
maps. This includes observations received from peers, just like the drone's
Visited Areas window. Green cells are searched; beige cells are unsearched.
The percentages and overlays follow the timeline when seeking backward.

A goal diamond identifies the global viewing position the drone selected, and
a dashed box identifies the patch it wants to inspect. The line between them
shows the viewing direction. This is the selected search objective, independent
of the combined movement waypoint that may also be influenced by cows.

The grid node publishes version 1 JSON in `std_msgs/String` on each drone's
`/search_state` topic (reliable, transient-local): simulation timestamp, map
geometry, searched cell indices, and a goal object or `null`. Cells are indexed
row-major from the lower-left map origin, with rows increasing in world Y.
The recorder stores the first observation time of each cell for each drone and
timestamped goal changes, avoiding a full grid copy at every replay frame.
`GET /api/runs/<id>/search?end=<seconds>` returns this history. Missing history
in older recordings is shown as **Not recorded**; it is never inferred from
flight paths or replaced by the final map. Search knowledge is published on
changes at the node's local-update cadence, including when display windows are
disabled.

## Optional live windows

The scenario editor's **Live simulation windows** section saves defaults for
Gazebo, RViz, occupancy grids, vector fields, Visited Areas, annotated detector
cameras and the cow tracker plot. All are off by default for unattended runs.
During an active test, expand the same section in its replay panel to change
these switches without restarting the mission. Grid/camera switches use ROS
parameters; Gazebo and RViz clients are owned and cleaned up by that run.
Settings changes are saved as timeline events. Recorded configuration retains
the original defaults.

Windows appear on the simulator's desktop (`DISPLAY`), not as embedded video
inside the browser. The Gazebo viewer uses installed/cached local models and a
local empty Insert catalog, so startup does not wait for Gazebo's online model
database. Online catalog browsing is disabled for this experiment viewer.
Verbose startup diagnostics are saved in `gazebo.log`.
If the runner has no desktop, it uses a virtual display and
the panel says so; use the browser's live trajectory/search view in that case.
Enabling extra windows can increase real elapsed time, while the test's
simulation-time limit stays the same.

`GET/POST /api/runs/<id>/visualizations` reads or requests the seven switches.
The response reports whether nodes have acknowledged the changes and any viewer
errors. Optional viewer logs are available in the Logs tab. Existing runs from
before this feature cannot acquire live controls mid-process; start a new run.

## Delete experiments

Use the trash button on an experiment to delete it, or select several (including
**Select all**) and choose **Delete selected**. The confirmation identifies the
count and makes clear that trajectories, metrics, configurations and logs will
be permanently removed. Saved scenarios and other experiments remain intact.
Running/finalizing tests must finish or be cancelled before deletion. Queued
tests may be deleted and will not start.

`DELETE /api/runs/<id>` deletes one experiment;
`POST /api/runs/delete` with `{"run_ids": ["id", "id"]}` deletes a selection.
Validation happens before any deletion, so a selection containing an active or
missing experiment is rejected as a whole. Data is staged while the catalog
transaction commits; interrupted transactions are recovered on startup.
