#!/usr/bin/env bash
# Stop Herding Lab and its container from any working directory.
set -euo pipefail
lab_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
exec python3 "$lab_repo_dir/scripts/stop_lab.py" "$@"
