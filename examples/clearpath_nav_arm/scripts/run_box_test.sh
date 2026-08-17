#!/usr/bin/env bash
set -e

mode=${1:-monitored}
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
case_root=$(cd -- "$script_dir/.." && pwd)

CASE_STUDY_RUNNER=box_scenario \
CASE_STUDY_RESULT_NAME="box_$mode" \
  exec "$case_root/scripts/run_case_study.sh" "$mode"
