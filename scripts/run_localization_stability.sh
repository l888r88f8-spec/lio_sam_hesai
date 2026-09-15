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

    # base_link -> hesai_lidar, mirroring the launch static transform. It has
    # to be applied to the reference pose before the tile index means anything.
    base_to_lidar_translation="0.02,0.0,0.32"

    reference_file="$results_dir/tiled-fault-reference.json"
    echo "capturing a no-fault reference pose"
    python3 test/integration/localization_regression.py \
      --bag "$bag_dir" \
      --map "$map_dir" \
      --params-template config/relocalization.yaml \
      --output-dir "$results_dir/reference" \
      --tiled-dir "$tiled_dir" \
      --baseline test/integration/baselines/hesai_sim.yaml \
      --record-reference "$reference_file"

    center_index=$(PYTHONPATH=test/integration python3 -c '
import json, sys, yaml
from stability_common import reference_base_to_lidar, tile_index_for_pose
reference = json.load(open(sys.argv[1], encoding="utf-8"))
manifest = yaml.safe_load(open(sys.argv[2], encoding="utf-8"))
base_T_lidar = reference_base_to_lidar(
    tuple(float(v) for v in sys.argv[3].split(",")))
index = tile_index_for_pose(reference["map_T_base"], base_T_lidar,
                            manifest["origin_xy"], manifest["tile_size_m"])
print("%d,%d" % index)
' "$reference_file" "$tiled_dir/manifest.yaml" "$base_to_lidar_translation")
    echo "reference tile index: $center_index"

    for fault_case in corrupt-manifest missing-center missing-neighbor; do
      case_root="$results_dir/$fault_case-map"
      case_tiled="$case_root/tiled"
      mkdir -p "$case_root"
      ln -s "$map_dir/GlobalMap.pcd" "$case_root/GlobalMap.pcd"
      ln -s "$map_dir/GroundMap.pcd" "$case_root/GroundMap.pcd"
      ln -s "$map_dir/sc_db.bin" "$case_root/sc_db.bin"

      fixture_info=$(PYTHONPATH=test/integration python3 -c '
import json, sys
from stability_common import prepare_tiled_fault_case
fixture = prepare_tiled_fault_case(
    sys.argv[1], sys.argv[2], sys.argv[3],
    center_index=tuple(int(v) for v in sys.argv[4].split(",")))
print(json.dumps({
    "fault_relative": fixture.fault_relative,
    "expected_index": fixture.expected_index,
}))
' "$tiled_dir" "$case_tiled" "$fault_case" "$center_index")

      fault_args=(
        --bag "$bag_dir"
        --map "$case_root"
        --params-template config/relocalization.yaml
        --output-dir "$results_dir/$fault_case"
        --tiled-dir "$case_tiled"
        --tiled-fault-case "$fault_case"
      )
      if [[ "$fault_case" != "corrupt-manifest" ]]; then
        expected_index=$(printf '%s' "$fixture_info" | python3 -c \
          'import json,sys; print(json.load(sys.stdin)["expected_index"])')
        fault_args+=(
          --tiled-fault-reference "$reference_file"
          --expected-missing-index "$expected_index"
        )
      fi
      python3 test/integration/localization_regression.py "${fault_args[@]}"
    done
    ;;
esac
