#!/usr/bin/env bash
set -eo pipefail

# This directory is mounted as /ros2_ws inside the simulation container.
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/iron/setup.bash
cd "$workspace_dir"
if [[ -t 1 ]]; then
    clear || true
fi
colcon build
# Refresh paths after the build so newly added ROS interfaces are available.
source "$workspace_dir/install/setup.bash"
python3 -c '
import ctypes
try:
    from cow_mapping_interfaces.msg import CowObservation
    ctypes.CDLL("libcow_mapping_interfaces__rosidl_typesupport_cpp.so")
except (ImportError, OSError) as exc:
    raise SystemExit(
        "Cow observation interfaces are unavailable. Build the workspace with "
        "colcon build, then run this launcher again.\n" + str(exc))
'
exec ros2 launch sjtu_drone_bringup bringup.launch.py "$@"
