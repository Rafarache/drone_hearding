#!/usr/bin/env bash
# Build and run the experiment app using this repository's existing ROS container.
set -euo pipefail
lab_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
lab_container="${SIM_TEST_CONTAINER:-drone_herding}"
lab_mode="${1:-serve}"
lab_workspace="$lab_repo_dir/ros_ws"
if [[ "$lab_mode" == "--help" || "$lab_mode" == "-h" ]]; then
  echo 'Usage: scripts/test_lab.sh [serve|setup|run BATCH.json]'
  echo 'Environment: SIM_TEST_CONTAINER (drone_herding), SIM_TEST_PORT (8765)'
  exit 0
fi
if [[ "$lab_mode" != serve && "$lab_mode" != setup && "$lab_mode" != run ]]; then
  echo "Unknown mode: $lab_mode" >&2; exit 2
fi
if [[ "$lab_mode" == serve ]] && python3 - "${SIM_TEST_PORT:-8765}" <<'PYHEALTH'
import json, sys, urllib.request
try:
    port = int(sys.argv[1])
    data = json.load(urllib.request.urlopen(f'http://127.0.0.1:{port}/api/health', timeout=1))
    assert data.get('ok') and data.get('data_dir') == '/ros2_ws/test_results' and data.get('max_duration_s') == 1200
except Exception:
    sys.exit(1)
PYHEALTH
then
  printf 'Herding Lab is already running: http://127.0.0.1:%s\n' "${SIM_TEST_PORT:-8765}"
  exit 0
fi
if ! docker inspect "$lab_container" >/dev/null 2>&1; then
  echo "Container $lab_container was not found. Start the repository ROS container first." >&2; exit 1
fi
if [[ "$(docker inspect "$lab_container" --format '{{.State.Running}}')" != true ]]; then
  docker start "$lab_container" >/dev/null
fi
if [[ "$(docker inspect "$lab_container" --format '{{.HostConfig.NetworkMode}}')" != host ]]; then
  echo 'The local app requires the ROS container to use --network host (the current repository setup).' >&2; exit 1
fi
# Desktop windows are opt-in per scenario. Xvfb supports cameras without DISPLAY.
docker exec "$lab_container" bash -lc '
  set -e
  source /opt/ros/iron/setup.bash
  bash /ros2_ws/src/simulation_test_pkg/scripts/setup_runtime.sh
  if [ ! -f /ros2_ws/.test_lab_venv/bin/pip ]; then
    echo "[Herding Lab] Creating the application Python environment..."
    python3 -m venv --system-site-packages /ros2_ws/.test_lab_venv
  fi
  echo "[Herding Lab] Installing application Python dependencies..."
  /ros2_ws/.test_lab_venv/bin/python -m pip install --disable-pip-version-check --timeout 20 --retries 2 -r /ros2_ws/src/simulation_test_pkg/requirements.lock
  cd /ros2_ws
  echo "[Herding Lab] Building ROS packages..."
  colcon build
'
if [[ "$lab_mode" != run ]]; then
  echo '[Herding Lab] Installing frontend dependencies...'
  npm ci --prefix "$lab_workspace/src/simulation_test_pkg/web" --no-audit --no-fund
  echo '[Herding Lab] Building the frontend...'
  npm run build --prefix "$lab_workspace/src/simulation_test_pkg/web"
fi
# Let the host user maintain top-level manifests even when Docker created the directory.
docker exec "$lab_container" bash -c 'mkdir -p /ros2_ws/test_results && chown "$1:$2" /ros2_ws/test_results' _ "$(id -u)" "$(id -g)"
# Keep a host-side provenance record because only ros_ws is mounted in Docker.
python3 - "$lab_repo_dir" <<'PY'
import json, pathlib, subprocess, sys
root=pathlib.Path(sys.argv[1]); target=root/'ros_ws/test_results'; target.mkdir(exist_ok=True)
def git(*args):
 return subprocess.check_output(['git','-C',str(root),*args],text=True).strip()
value={'revision':git('rev-parse','HEAD'),'dirty':bool(git('status','--porcelain')),
       'working_tree_patch':git('diff','HEAD'),
       'untracked_files':git('ls-files','--others','--exclude-standard').splitlines()}
(target/'checkout_provenance.json').write_text(json.dumps(value,indent=2))
PY
if [[ "$lab_mode" == setup ]]; then
  echo 'Setup complete. Run scripts/test_lab.sh to open the local experiment app.'; exit 0
fi
if [[ "$lab_mode" == run ]]; then
  if [[ -z "${2:-}" || ! -f "$2" ]]; then echo 'Supply an existing batch JSON file.' >&2; exit 2; fi
  lab_spec="/ros2_ws/test_results/batch-request-$(date +%s)-$$.json"
  docker cp "$2" "$lab_container:$lab_spec"
  exec docker exec -i "$lab_container" bash -lc 'source /opt/ros/iron/setup.bash && source /ros2_ws/install/setup.bash && cd /ros2_ws && exec .test_lab_venv/bin/python -m simulation_test_pkg.cli --spec "$1"' _ "$lab_spec"
fi
printf 'Herding Lab: http://127.0.0.1:%s\n' "${SIM_TEST_PORT:-8765}"
echo '[Herding Lab] Starting the server. This terminal stays open; use Ctrl+C to stop it.'
exec python3 "$lab_repo_dir/scripts/test_lab_service.py" --container "$lab_container" --port "${SIM_TEST_PORT:-8765}"
