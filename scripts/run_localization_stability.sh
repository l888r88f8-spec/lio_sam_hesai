#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 {quick|record-baseline|memory|tf-continuity|tiled-generate|tiled-quick|tiled-faults}" >&2
}

if [[ $# -ne 1 ]]; then
  usage
  exit 64
fi

mode=$1
case "$mode" in
  quick|record-baseline|memory|tf-continuity|tiled-generate|tiled-quick|tiled-faults) ;;
  *)
    usage
    exit 64
    ;;
esac

data_root=${LIO_SAM_TESTDATA_ROOT:-/home/robot}
workspace_root=${LIO_SAM_WORKSPACE:-/home/robot/slam_ws}
install_prefix=${LIO_SAM_INSTALL_PREFIX:-}
bag_dir="$data_root/rosbag2/hesai_sim"
map_dir="$data_root/map/hesai_sim2"

if [[ "$mode" != "tiled-generate" && ! -d "$bag_dir" ]]; then
  echo "required rosbag directory is missing: $bag_dir" >&2
  exit 2
fi
if [[ ! -d "$map_dir" ]]; then
  echo "required map directory is missing: $map_dir" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
if [[ -n "$install_prefix" ]]; then
  source "$install_prefix/share/lio_sam_hesai/local_setup.bash"
else
  source "$workspace_root/install/setup.bash"
fi
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
tiled_dir="$map_dir/tiled"

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
  tiled-generate)
    ros2 run lio_sam_hesai lio_sam_hesai_generate_tiled_map "$map_dir"
    ;;
  tiled-quick)
    if [[ ! -f "$tiled_dir/manifest.yaml" ]]; then
      echo "required tiled manifest is missing: $tiled_dir/manifest.yaml" >&2
      exit 2
    fi
    python3 test/integration/localization_regression.py \
      "${common_args[@]}" \
      --tiled-dir "$tiled_dir" \
      --baseline test/integration/baselines/hesai_sim.yaml
    ;;
  tiled-faults)
    if [[ ! -f "$tiled_dir/manifest.yaml" ]]; then
      echo "required tiled manifest is missing: $tiled_dir/manifest.yaml" >&2
      exit 2
    fi
    for fault_case in corrupt-manifest missing-center missing-neighbor; do
      case_root="$results_dir/$fault_case-map"
      case_tiled="$case_root/tiled"
      mkdir -p "$case_root"
      ln -s "$map_dir/GlobalMap.pcd" "$case_root/GlobalMap.pcd"
      ln -s "$map_dir/GroundMap.pcd" "$case_root/GroundMap.pcd"
      ln -s "$map_dir/sc_db.bin" "$case_root/sc_db.bin"
      center_args=()
      if [[ -n "${LIO_SAM_TILED_FAULT_CENTER_INDEX:-}" ]]; then
        center_args+=("$LIO_SAM_TILED_FAULT_CENTER_INDEX")
      fi
      missing_index=$(PYTHONPATH=test/integration python3 -c \
        'import sys; from stability_common import prepare_tiled_fault_case; center=tuple(map(int, sys.argv[4].split(","))) if len(sys.argv) > 4 else None; print(prepare_tiled_fault_case(sys.argv[1], sys.argv[2], sys.argv[3], center))' \
        "$tiled_dir" "$case_tiled" "$fault_case" "${center_args[@]}")
      fault_args=(
        --bag "$bag_dir"
        --map "$case_root"
        --params-template config/relocalization.yaml
        --output-dir "$results_dir/$fault_case"
        --tiled-dir "$case_tiled"
        --tiled-fault-case "$fault_case"
      )
      if [[ "$fault_case" == "missing-neighbor" ]]; then
        fault_args+=(--expected-missing-index "$missing_index")
      fi
      python3 test/integration/localization_regression.py "${fault_args[@]}"
    done
    ;;
esac
