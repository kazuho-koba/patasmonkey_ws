#!/usr/bin/env bash
# 通常mapperの使用TFと、同じ再生の通過経路poseを保存する。
# 前回の融合/RViz用launch・YAMLには触れない。実機センサ・モータは起動しない。
set -eo pipefail
if [ "$#" -lt 2 ]; then
  echo "usage: bash $0 BAG_DIR OUTPUT_DIR [launch引数:=値 ...]" >&2
  exit 2
fi
bag=$1
output=$2
shift 2
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
source /workspaces/patasmonkey_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID=${MAPPER_TRACE_DOMAIN_ID:-121}
export MAPPER_TRACE_DIRECTORY="$output/trace"
export ROS_LOG_DIR="$output/ros_logs"
mkdir -p "$ROS_LOG_DIR"
launch_pid=
recorder_pid=
cleanup() {
  # 自分が起動したPIDのみSIGINTで停止し、ファイルclose・子node終了を待つ。
  if [ -n "$recorder_pid" ]; then
    kill -INT "$recorder_pid" 2>/dev/null || true
    wait "$recorder_pid" || true
  fi
  if [ -n "$launch_pid" ]; then
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" || true
  fi
}
trap cleanup EXIT
# bashのbackground jobから継承されるSIGINT ignoreをexec前に解除する。
# 専用process groupを作り、launch自身のSIGINT shutdownで子nodeも終了させる。
# 追加引数を配列のまま渡す。値の再解釈やshell展開は行わない。
setsid python3 -c 'import os,signal,sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp("ros2", ["ros2", "launch", "pm_perception", "mapper_tf_trace_replay.launch.py", "trace_output_dir:="+os.environ["MAPPER_TRACE_DIRECTORY"]]+sys.argv[1:])' "$@" \
  > "$output/localization.log" 2>&1 &
launch_pid=$!
sleep 3
setsid python3 /workspaces/patasmonkey_ws/src/pm_evaluation/tools/record_recomputed_local_poses.py \
  --output "$output/recomputed_poses.csv" > "$output/pose_recorder.log" 2>&1 &
recorder_pid=$!
sleep 2
# 実効設定を保存して、検証側の閾値・セル幅不一致を防ぐ。
ros2 param dump /depth_elevation_mapper --output-dir "$output" > "$output/parameter_dump.log"
ros2 run pm_evaluation bag_clock_player "$bag" --rate 1.0 \
  --topic /wheel/odometry --topic /vio/odometry --topic /wit/imu --topic /tf_static --topic /oak/depth/image_raw --topic /oak/depth/camera_info \
  > "$output/player.log" 2>&1
sleep 2
