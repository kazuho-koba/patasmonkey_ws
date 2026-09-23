#!/usr/bin/env bash
# Foxy container内で、最新localization・terrain mapper・経路評価をまとめて実行する。
# 使用例: bash src/pm_evaluation/tools/run_traversed_terrain_replay.sh BAG_DIR OUTPUT_DIR [FORENSIC_DIR]
set -eo pipefail

if [ "$#" -lt 2 ] || [ "$#" -gt 3 ]; then
  echo "usage: $0 BAG_DIR OUTPUT_DIR [FORENSIC_DIR]" >&2
  exit 2
fi
bag_dir=$1
result_dir=$2
forensic_dir=${3:-}
if [ ! -d "$bag_dir" ]; then
  echo "bag directory not found: $bag_dir" >&2
  exit 2
fi
mkdir -p "$result_dir"

# Foxyのsetup scriptには未設定変数を参照するものがあるため、source後にnounsetを使う。
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
source /workspaces/patasmonkey_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID=${TERRAIN_EVAL_DOMAIN_ID:-97}
export ROS_LOG_DIR="$result_dir/ros_logs"
mkdir -p "$ROS_LOG_DIR"

launch_args=()
if [ -n "$forensic_dir" ]; then
  # 第3引数がある場合だけ、低速なCSV・pixel由来情報の保存を明示的に有効化する。
  launch_args+=("terrain_forensic_output_dir:=$forensic_dir")
  forensic_half_width=${TERRAIN_FORENSIC_ROI_HALF_WIDTH_M:-0.60}
  launch_args+=("terrain_forensic_roi_half_width_m:=$forensic_half_width")
  if [ -n "${TERRAIN_FORENSIC_TARGETS_CSV:-}" ]; then
    launch_args+=("terrain_forensic_targets_csv:=$TERRAIN_FORENSIC_TARGETS_CSV")
  fi
fi
setsid ros2 launch pm_perception terrain_mapping_latest_localization_replay.launch.py \
  "${launch_args[@]}" > "$result_dir/launch.log" 2>&1 &
launch_pid=$!
collector_pid=
cleanup() {
  # このscriptが起動したPID/PGIDだけを対象に停止する。別のROS作業は触らない。
  if [ -n "$collector_pid" ]; then
    kill -INT "$collector_pid" 2>/dev/null || true
    wait "$collector_pid" 2>/dev/null || true
  fi
  kill -TERM -- -"$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT

# launchが購読・TF listenerを準備した後にcollectorを起動し、その購読成立後にbagを流す。
sleep 4
python3 /workspaces/patasmonkey_ws/src/pm_evaluation/tools/evaluate_traversed_terrain.py \
  --output "$result_dir" --duration 300 \
  > "$result_dir/evaluator.log" 2>&1 &
collector_pid=$!
sleep 3
ros2 run pm_evaluation bag_clock_player "$bag_dir" --rate 1.0 \
  --topic /wheel/odometry --topic /vio/odometry --topic /wit/imu \
  --topic /tf_static --topic /oak/depth/image_raw \
  > "$result_dir/player.log" 2>&1
sleep 3
kill -INT "$collector_pid" 2>/dev/null || true
wait "$collector_pid"
collector_pid=
python3 /workspaces/patasmonkey_ws/src/pm_evaluation/tools/plot_traversed_terrain.py \
  "$result_dir/path_samples.csv" \
  --output "$result_dir/traversed_hazard.png"
sed -n '1,160p' "$result_dir/summary.json"
