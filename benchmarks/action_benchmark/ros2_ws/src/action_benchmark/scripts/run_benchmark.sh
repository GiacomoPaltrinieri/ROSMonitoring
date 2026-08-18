#!/usr/bin/env bash
set -eo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
package_dir=$(cd -- "$script_dir/.." && pwd)
workspace=$(cd -- "$package_dir/../.." && pwd)
rosmonitoring=$(cd -- "$workspace/../../.." && pwd)

n=${1:?"Usage: $0 N GOALS ORDER"}
goals=${2:?"Usage: $0 N GOALS ORDER"}
order=${3:?"Usage: $0 N GOALS ORDER"}

source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd "$workspace"

if [[ ${ACTION_BENCHMARK_SKIP_PREPARE:-0} != 1 ]]; then
    colcon build --packages-select action_benchmark >/dev/null
    (
        generator_copy=$(mktemp -d)
        trap 'rm -rf "$generator_copy"' EXIT
        cp -a "$rosmonitoring/generator/ros2_devel/." "$generator_copy/"
        cd "$generator_copy"
        python3 generator \
            --config "$package_dir/config/rosmonitoring_n$n.yaml" >/dev/null
    )
    colcon build --packages-select action_benchmark monitor >/dev/null
fi
source install/setup.bash

oracle_pid=
monitor_pid=
cleanup() {
    [[ -z $monitor_pid ]] || kill "$monitor_pid" 2>/dev/null || true
    [[ -z $oracle_pid ]] || kill "$oracle_pid" 2>/dev/null || true
    [[ -z $monitor_pid ]] || wait "$monitor_pid" 2>/dev/null || true
    [[ -z $oracle_pid ]] || wait "$oracle_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

measure() {
    ros2 launch "$1" n:="$n" goals:="$goals" order:="$order" \
        repetitions:=1 |
        sed -n 's/.*Duration=\([0-9.]*\).*/\1/p'
}

baseline=$(measure "$package_dir/launch/run.launch")

python3 "$rosmonitoring/oracle/TLOracle/always_true_oracle.py" \
    --port 8080 >/dev/null 2>&1 &
oracle_pid=$!
for _ in {1..50}; do
    ss -ltnH 'sport = :8080' | grep -q . && break
    sleep 0.1
done
ss -ltnH 'sport = :8080' | grep -q . || {
    echo "Oracle not ready" >&2
    exit 1
}

"$workspace/install/monitor/lib/monitor/action_benchmark_monitor" \
    >/dev/null 2>&1 &
monitor_pid=$!
monitored=$(measure "$package_dir/launch/run_instrumented.launch")

[[ -n $baseline && -n $monitored ]] || {
    echo "Missing benchmark result" >&2
    exit 1
}
echo "$baseline,$monitored"
