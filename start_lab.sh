#!/usr/bin/env bash
# Start the ROS container and prepare/run Herding Lab from any working directory.
set -euo pipefail
lab_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
lab_workspace="$(cd -- "$lab_repo_dir/ros_ws" && pwd -P)"
lab_container="${SIM_TEST_CONTAINER:-drone_herding}"
lab_image="${SIM_TEST_IMAGE:-drone_herding:1.0}"
lab_mode="${1:-serve}"

if [[ "$lab_mode" == --help || "$lab_mode" == -h ]]; then
  cat <<'HELP'
Usage: ./start_lab.sh [serve|setup|run BATCH.json]

Starts or creates the ROS container, installs dependencies, and builds the lab.
The default mode serves Herding Lab at http://127.0.0.1:8765.
An existing healthy server is reused. Ctrl+C stops the server; the container stays up.
Run ./stop_lab.sh to shut down both the server and container.

Environment:
  SIM_TEST_IMAGE      Docker image (default: drone_herding:1.0); built if missing
  SIM_TEST_CONTAINER  Container name (default: drone_herding)
  SIM_TEST_PORT       Lab port (default: 8765)

Requires Docker, Python 3, Git, and Node.js/npm on the host.
HELP
  exit 0
fi
if [[ "$lab_mode" != serve && "$lab_mode" != setup && "$lab_mode" != run ]]; then
  echo "Unknown mode: $lab_mode. Use --help for usage." >&2
  exit 2
fi
if [[ "$lab_mode" == run && ( -z "${2:-}" || ! -f "$2" ) ]]; then
  echo 'Supply an existing batch JSON file: ./start_lab.sh run BATCH.json' >&2
  exit 2
fi
for lab_command in docker python3 git; do
  if ! command -v "$lab_command" >/dev/null; then
    echo "Required host command not found: $lab_command" >&2
    exit 1
  fi
done
if [[ "$lab_mode" != run ]] && ! command -v npm >/dev/null; then
  echo 'Install Node.js 20.19+ or 22.12+ (including npm) before starting the lab.' >&2
  exit 1
fi
if ! docker info >/dev/null; then
  echo 'Docker is unavailable. Start Docker and ensure your user can access it.' >&2
  exit 1
fi

if lab_inspection="$(docker container inspect "$lab_container" 2>/dev/null)"; then
  # Refuse to build into a similarly named container belonging to another checkout.
  lab_running="$(python3 -c '
import json, pathlib, sys
container = json.load(sys.stdin)[0]
if container["HostConfig"]["NetworkMode"] != "host":
    sys.exit("The existing container must use --network host. Choose another SIM_TEST_CONTAINER.")
mount = next((m for m in container.get("Mounts", []) if m["Destination"] == "/ros2_ws"), {})
if (mount.get("Type") != "bind" or not mount.get("RW") or
        pathlib.Path(mount["Source"]).resolve() != pathlib.Path(sys.argv[1]).resolve()):
    sys.exit("The existing container must mount this checkout at /ros2_ws with write access. Choose another SIM_TEST_CONTAINER.")
print("true" if container["State"]["Running"] else "false")
' "$lab_workspace" <<< "$lab_inspection")"
  if [[ "$lab_running" != true ]]; then
    printf 'Starting container %s...\n' "$lab_container"
    docker start "$lab_container" >/dev/null
  fi
else
  if ! docker image inspect "$lab_image" >/dev/null 2>&1; then
    printf 'Building missing Docker image %s...\n' "$lab_image"
    docker build --tag "$lab_image" "$lab_repo_dir"
  fi
  lab_docker_args=(
    run --detach --init --name "$lab_container" --network host
    --mount "type=bind,source=$lab_workspace,target=/ros2_ws"
    --workdir /ros2_ws
    --env ROS_DOMAIN_ID=11 --env TURTLEBOT3_MODEL=burger
    --env QT_X11_NO_MITSHM=1 --env GAZEBO_MODEL_PATH=/ros2_ws/models
  )
  # Use the host display when its authorization cookie is available. Otherwise
  # the lab provisions Xvfb for camera rendering without a desktop session.
  lab_xauth="${XAUTHORITY:-$HOME/.Xauthority}"
  if [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix && -f "$lab_xauth" ]]; then
    lab_docker_args+=(
      --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix,readonly
      --mount "type=bind,source=$lab_xauth,target=/tmp/lab.xauthority,readonly"
      --env "DISPLAY=$DISPLAY" --env XAUTHORITY=/tmp/lab.xauthority
    )
  else
    echo 'Using a virtual display for simulation cameras.'
  fi
  if [[ -d /dev/dri ]]; then
    lab_docker_args+=(--device /dev/dri:/dev/dri)
  fi
  printf 'Creating container %s from %s...\n' "$lab_container" "$lab_image"
  docker "${lab_docker_args[@]}" "$lab_image" sleep infinity >/dev/null
fi

for ((lab_attempt=0; lab_attempt<30; lab_attempt++)); do
  if docker exec "$lab_container" true >/dev/null 2>&1; then
    exec "$lab_repo_dir/scripts/test_lab.sh" "$@"
  fi
  sleep 0.2
done
echo "Container $lab_container did not become ready. Check: docker logs $lab_container" >&2
exit 1
