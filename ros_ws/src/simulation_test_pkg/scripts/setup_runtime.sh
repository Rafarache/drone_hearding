#!/usr/bin/env bash
# Shared by terminal setup and the lab's Build and reload action.
set -euo pipefail

echo '[Herding Lab] Checking ROS runtime packages...'
runtime_packages=(
  python3-venv xvfb xauth
  "ros-${ROS_DISTRO:-iron}-xacro"
  "ros-${ROS_DISTRO:-iron}-robot-state-publisher"
  "ros-${ROS_DISTRO:-iron}-joint-state-publisher"
)
missing_packages=()
for package in "${runtime_packages[@]}"; do
  if [[ "$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true)" != 'install ok installed' ]]; then
    missing_packages+=("$package")
  fi
done
if ((${#missing_packages[@]})); then
  printf '[Herding Lab] Installing missing packages: %s\n' "${missing_packages[*]}"
  apt-get update -qq
  DEBIAN_FRONTEND=noninteractive apt-get install -y -qq --no-install-recommends "${missing_packages[@]}"
fi

# ROS entry points use system Python. Check SciPy there, including NumPy
# compatibility, rather than only inside the lab server's virtual environment.
echo '[Herding Lab] Checking SciPy and NumPy compatibility...'
if ! python3 -W error -c 'from scipy.optimize import linear_sum_assignment' 2>/dev/null; then
  echo '[Herding Lab] Installing SciPy (requires a package download)...'
  python3 -m pip install --disable-pip-version-check --timeout 20 --retries 2 --no-cache-dir --upgrade 'numpy<2' scipy
fi
echo '[Herding Lab] ROS runtime dependencies are ready.'
