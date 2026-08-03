#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 {quick|record-baseline|memory|tf-continuity}" >&2
}

if [[ $# -ne 1 ]]; then
  usage
  exit 64
fi

mode=$1
case "$mode" in
  quick|record-baseline|memory|tf-continuity) ;;
  *)
    usage
    exit 64
    ;;
esac

data_root=${LIO_SAM_TESTDATA_ROOT:-/home/robot}
workspace_root=${LIO_SAM_WORKSPACE:-/home/robot/slam_ws}
bag_dir="$data_root/rosbag2/hesai_sim"
map_dir="$data_root/map/hesai_sim2"

if [[ ! -d "$bag_dir" ]]; then
  echo "required rosbag directory is missing: $bag_dir" >&2
  exit 2
fi
if [[ ! -d "$map_dir" ]]; then
  echo "required map directory is missing: $map_dir" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "$workspace_root/install/setup.bash"
set -u

results_dir=$(mktemp -d /tmp/lio-sam-hesai-stability.XXXXXX)
if [[ "${LIO_SAM_KEEP_RESULTS:-}" != "1" ]]; then
  trap 'rm -rf "$results_dir"' EXIT
fi
printf 'results_dir=%s\n' "$results_dir"
if [[ -n "${GITHUB_OUTPUT:-}" ]]; then
  printf 'results_dir=%s\n' "$results_dir" >> "$GITHUB_OUTPUT"
fi

common_args=(
  --bag "$bag_dir"
  --map "$map_dir"
  --params-template config/relocalization.yaml
  --output-dir "$results_dir"
)

case "$mode" in
  quick)
    python3 test/integration/localization_regression.py \
      "${common_args[@]}" \
      --baseline test/integration/baselines/hesai_sim.yaml
    ;;
  record-baseline)
    python3 test/integration/localization_regression.py \
      "${common_args[@]}" \
      --record-baseline
    ;;
  memory)
    python3 test/integration/localization_rss.py \
      "${common_args[@]}" \
      --baseline test/integration/baselines/hesai_sim.yaml
    ;;
  tf-continuity)
    python3 test/integration/localization_tf_continuity.py \
      "${common_args[@]}"
    ;;
esac
