#!/usr/bin/env bash
set -eo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
case_root=$(cd -- "$script_dir/.." && pwd)
required_packages=(
  ros-jazzy-clearpath-simulator
  ros-jazzy-clearpath-nav2-demos
  ros-jazzy-clearpath-manipulators
  ros-jazzy-kortex-description
  ros-jazzy-rmw-cyclonedds-cpp
)

missing_packages=()
for package in "${required_packages[@]}"; do
  if ! dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -q 'install ok installed'; then
    missing_packages+=("$package")
  fi
done

if ((${#missing_packages[@]})); then
  echo "Missing system packages: ${missing_packages[*]}"
  echo "Run this command yourself, then execute bootstrap.sh again:"
  echo "  sudo apt update && sudo apt install ${missing_packages[*]}"
  exit 2
fi

source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if ! python3 "$case_root/scripts/check_ros_runtime.py"; then
  echo "CycloneDDS could not initialize the ROS action runtime." >&2
  exit 4
fi

if [[ ! -x "$case_root/.venv/bin/python" ]]; then
  python3 -m venv "$case_root/.venv"
fi
"$case_root/.venv/bin/python" -m pip install \
  --disable-pip-version-check \
  -r "$case_root/oracle/requirements.txt"

"$case_root/scripts/generate_monitor.sh"

cd "$case_root"
colcon build --symlink-install --cmake-clean-cache --packages-select \
  rosmonitoring_interfaces monitor nav_arm_case_study

echo
echo "Bootstrap complete. Source the workspace with:"
echo "  source $case_root/install/setup.bash"
