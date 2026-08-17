#!/usr/bin/env bash
set -eo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
case_root=$(cd -- "$script_dir/.." && pwd)
project_root=$(cd -- "$case_root/../.." && pwd)
monitor_dir="$case_root/src/monitor"
runtime_config=$(mktemp)
generator_work=$(mktemp -d)

cleanup() {
  rm -f -- "$runtime_config"
  if [[ "$generator_work" == /tmp/tmp.* ]]; then
    rm -rf -- "$generator_work"
  fi
}
trap cleanup EXIT

source /opt/ros/jazzy/setup.bash

if [[ "$monitor_dir" != "$case_root/src/monitor" ]]; then
  echo "Refusing to remove unexpected monitor path: $monitor_dir" >&2
  exit 3
fi
if [[ -d "$monitor_dir" ]]; then
  rm -rf -- "$monitor_dir"
fi

mkdir -p "$generator_work/code"
cp -a "$project_root/generator/ros2_devel/generator" \
  "$project_root/generator/ros2_devel/generator.py" "$generator_work/"
cp -a "$project_root/generator/ros2_devel/code/monitor" \
  "$project_root/generator/ros2_devel/code/rosmonitoring_interfaces" \
  "$generator_work/code/"
find "$generator_work/code/monitor/monitor" \
  -maxdepth 1 -type f -name '*_monitor.py' -delete

cd "$generator_work"
sed \
  -e "s|^path: src/$|path: $case_root/src/|" \
  "$case_root/config/rosmonitor.yaml" > "$runtime_config"
python3 generator --config_file "$runtime_config"

test -f "$monitor_dir/monitor/nav_arm_interlock_monitor.py"
echo "Generated monitor in $monitor_dir"
