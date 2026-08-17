#!/usr/bin/env bash
set -eo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
case_root=$(cd -- "$script_dir/.." && pwd)
project_root=$(cd -- "$case_root/../.." && pwd)
mode=${1:-monitored}
scenario=${CASE_STUDY_RUNNER:-scenario_runner}
result_name=${CASE_STUDY_RESULT_NAME:-$mode}
state_file=/tmp/clearpath_nav_arm.sessions
launch_sid=
runner_sid=

exec 9>"/tmp/clearpath_nav_arm.lock"
if ! flock -n 9; then
  echo "Another Clearpath case-study run is already active." >&2
  echo "Wait for it to finish or stop it with Ctrl-C before starting a new run." >&2
  exit 3
fi

if [[ "$mode" != "monitored" && "$mode" != "baseline" ]]; then
  echo "Usage: $0 [monitored|baseline]" >&2
  exit 2
fi

source /opt/ros/jazzy/setup.bash
source "$case_root/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$case_root/config/cyclonedds.xml"
export CASE_STUDY_ROOT="$case_root"
export ROSMONITORING_ROOT="$project_root"
mkdir -p "$case_root/results"
cd "$case_root"

session_is_alive() {
  local session_id=$1
  [[ "$session_id" =~ ^[1-9][0-9]*$ ]] || return 1
  ps -eo sid= | awk -v target="$session_id" '
    $1 == target { found = 1; exit }
    END { exit !found }
  '
}

session_is_ours() {
  local session_id=$1
  ps -eo sid=,args= | awk -v target="$session_id" -v root="$case_root" '
    $1 == target && (index($0, root) || /nav_arm_case_study/) { found = 1 }
    END { exit !found }
  '
}

stop_session() {
  local label=$1
  local session_id=$2

  session_is_alive "$session_id" || return 0

  # Gazebo and some ROS nodes create their own process groups, but remain in
  # the session created below. Signal the complete session, not only its
  # original process group.
  pkill -INT -s "$session_id" 2>/dev/null || true
  for _ in {1..15}; do
    session_is_alive "$session_id" || return 0
    sleep 1
  done

  echo "$label did not stop after 15s; terminating its complete session" >&2
  pkill -TERM -s "$session_id" 2>/dev/null || true
  for _ in {1..5}; do
    session_is_alive "$session_id" || return 0
    sleep 1
  done

  if session_is_alive "$session_id"; then
    echo "$label still did not stop; killing its complete session" >&2
    pkill -KILL -s "$session_id" 2>/dev/null || true
  fi
}

# Recover automatically if a previous invocation was closed abruptly.
if [[ -f "$state_file" ]]; then
  while read -r stale_sid; do
    if session_is_ours "$stale_sid"; then
      stop_session "Previous case-study run" "$stale_sid"
    elif session_is_alive "$stale_sid"; then
      echo "Stored session $stale_sid is not ours; leaving it untouched." >&2
    fi
  done < "$state_file"
  rm -f "$state_file"
fi

existing_nodes=$(timeout 5s ros2 node list --no-daemon 2>/dev/null || true)
if grep -q '^/a200_0000/' <<<"$existing_nodes"; then
  echo "A different simulation is already using /a200_0000; leaving it untouched." >&2
  grep '^/a200_0000/' <<<"$existing_nodes" >&2
  exit 4
fi

cleanup() {
  trap - EXIT INT TERM HUP
  stop_session "Scenario runner" "$runner_sid"
  stop_session "ROS/Gazebo launch" "$launch_sid"
  [[ -z "$runner_sid" ]] || wait "$runner_sid" 2>/dev/null || true
  [[ -z "$launch_sid" ]] || wait "$launch_sid" 2>/dev/null || true
  rm -f "$state_file"
}
trap cleanup EXIT INT TERM HUP

setsid ros2 launch nav_arm_case_study "$mode.launch.py" 9>&- &
launch_sid=$!
printf '%s\n' "$launch_sid" > "$state_file"

runner=(
  ros2 run nav_arm_case_study "$scenario"
  --monitoring "$([[ "$mode" == monitored ]] && echo enabled || echo disabled)"
  --result-file "$case_root/results/$result_name.json"
  --ros-args
  -r /tf:=/a200_0000/tf
  -r /tf_static:=/a200_0000/tf_static
)

if [[ "$mode" == "monitored" ]]; then
  runner+=(
    -r /a200_0000/navigate_to_pose/_action/send_goal:=/a200_0000/navigate_to_pose/_action/send_goal_mon
    -r /a200_0000/arm_0_gripper_controller/gripper_cmd/_action/send_goal:=/a200_0000/arm_0_gripper_controller/gripper_cmd/_action/send_goal_mon
  )
fi

setsid timeout --signal=INT 360s "${runner[@]}" 9>&- &
runner_sid=$!
printf '%s\n' "$launch_sid" "$runner_sid" > "$state_file"

set +e
wait "$runner_sid"
runner_status=$?
set -e
exit "$runner_status"
