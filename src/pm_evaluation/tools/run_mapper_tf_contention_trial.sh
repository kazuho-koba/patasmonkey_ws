#!/usr/bin/env bash
# Jetsonホストで1条件ずつ実行する。SSH制御sessionを維持したまま完了まで待つ。
# 第2引数はTF thread、第6引数はdepth reliability、第7引数はQoS history depth。
# pending_queue_size（TF待ちqueue）は変更しない。各比較では1因子だけ変え、
# 全topic bag負荷を共通にする。probe比較は主A/Bと分けて解釈する。
set -eo pipefail
label="${1:?trial label required}"
dedicated="${2:?false or true required}"
duration="${3:-60}"
probe="${4:-false}"
compare_reliable="${5:-false}"
mapper_reliability="${6:-best_effort}"
mapper_history_depth="${7:-3}"
[[ "$mapper_history_depth" =~ ^[1-9][0-9]*$ ]] || exit 2
case "$mapper_reliability" in best_effort|reliable) ;; *) exit 2 ;; esac
case "$dedicated" in true|false) ;; *) echo 'dedicated must be true/false' >&2; exit 2 ;; esac
case "$probe" in true|false|toggle) ;; *) echo 'probe must be true/false/toggle' >&2; exit 2 ;; esac
case "$compare_reliable" in true|false) ;; *) exit 2 ;; esac
[[ "$label" =~ ^[a-zA-Z0-9_-]+$ ]] || exit 2
[[ "$duration" =~ ^[1-9][0-9]*$ ]] || exit 2
if [ "$probe" = toggle ]; then
    [ "$duration" -ge 30 ] && [ $((duration % 3)) -eq 0 ] || exit 2
fi
task_workspace="${PM_TRIAL_WORKSPACE:-/home/nvidia/patasmonkey_ws}"
task_ros_workspace="${PM_TRIAL_ROS_WORKSPACE:-/home/nvidia/ros2_ws}"
source /opt/ros/foxy/setup.bash
source "$task_ros_workspace/install/setup.bash"
source "$task_workspace/install/setup.bash"

# 他のlive試験を重ねて比較条件を変えない。既存processには一切signalを送らない。
if pgrep -f '(^|/)(depth_elevation_mapper_node|oakd_vio_rgbd_node|run_subscribe_msckf)( |$)|ros2 bag record' >/dev/null; then
    echo '既存mapper/camera/VO/recorderが動いています。試験を開始しません。' >&2
    exit 3
fi
task_stamp="$(date +%Y%m%d_%H%M%S)"
run_dir="/tmp/mapper_tf_contention_${task_stamp}_${label}"
bag_name="rosbag2_$(date +%Y_%m_%d-%H_%M_%S)_${label}"
bag_dir="$task_workspace/bags/$bag_name"
mkdir "$run_dir"
printf 'label=%s\ndedicated_thread=%s\nbag_dir=%s\nprobe=%s\ncompare_reliable=%s\nmapper_reliability=%s\nmapper_history_depth=%s\n' \
    "$label" "$dedicated" "$bag_dir" "$probe" "$compare_reliable" "$mapper_reliability" "$mapper_history_depth" > "$run_dir/trial.txt"
python3 -c 'import os,signal,sys; signal.signal(signal.SIGINT,signal.SIG_DFL); os.execvp(sys.argv[1],sys.argv[1:])' \
    ros2 launch pm_bringup pm_bag_global_localization.launch.py \
    use_teleop:=false use_vehicle_interface:=false use_oakd:=true use_openvins:=true \
    use_gnss:=true use_ntrip:=true record_bag:=true localization_mode:=legacy \
    wit_timing_diagnostics:=false \
    mapper_callback_diagnostics:=true mapper_executor_diagnostics:=true \
    mapper_executor_diagnostics_csv:="$run_dir/executor.csv" \
    mapper_tf_listener_dedicated_thread:="$dedicated" \
    mapper_depth_subscription_queue_depth:="$mapper_history_depth" mapper_tf_retry_rate_hz:=100.0 \
    mapper_depth_subscription_reliability:="$mapper_reliability" \
    mapper_debug_publish_rate:=2.0 mapper_publish_stage2_debug_layers:=true \
    bag_name:="$bag_name" > "$run_dir/launch.log" 2>&1 &
launch_pid=$!
printf 'launch_pid=%s\nrun_dir=%s\n' "$launch_pid" "$run_dir" | tee -a "$run_dir/trial.txt"
monitor_pid=''
tegra_pid=''
top_pid=''
probe_pid=''

finish_trial() {
    # launchへSIGINTを送り、remote sessionを切らずに子recorderの確定まで待つ。
    trap '' INT TERM
    if [ -n "$probe_pid" ]; then
        kill -INT "$probe_pid" 2>/dev/null || true
        wait "$probe_pid" 2>/dev/null || true
    fi
    # 停止前に今回のlaunchの子孫PIDだけ保存する。孤児化したVOやrecorderを、
    # 後から全systemの同名process検索で止めないようにする。
    child_pids="$(python3 - "$launch_pid" <<'PY'
import pathlib, sys
root = int(sys.argv[1])
parents, commands = {}, {}
for item in pathlib.Path('/proc').glob('[0-9]*'):
    try:
        status = (item / 'status').read_text()
        parent = next(int(line.split()[1]) for line in status.splitlines() if line.startswith('PPid:'))
        pid = int(item.name)
        parents[pid] = parent
        commands[pid] = (item / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
    except (OSError, StopIteration):
        continue
for pid, command in commands.items():
    parent, visited = pid, set()
    while parent in parents and parent not in visited and parent != root:
        visited.add(parent)
        parent = parents[parent]
    if parent == root and ('run_subscribe_msckf --ros-args' in command or 'ros2 bag record ' in command):
        print(pid)
PY
)"
    kill -INT "$launch_pid" 2>/dev/null || true
    # launchが停止待ちで固まっても、30秒で今回のrecorderへ再度SIGINTを送る。
    for ((i=0; i<30; i++)); do
        kill -0 "$launch_pid" 2>/dev/null || break
        sleep 1
    done
    for pid in $child_pids; do
        if [ -r "/proc/$pid/cmdline" ]; then
            command="$(tr '\0' ' ' < "/proc/$pid/cmdline")"
            case "$command" in *run_subscribe_msckf*|*"ros2 bag record "*) kill -INT "$pid" 2>/dev/null || true ;; esac
        fi
    done
    wait "$launch_pid" || true
    for pid in $child_pids; do
        while kill -0 "$pid" 2>/dev/null; do sleep 1; done
    done
    if [ -n "$monitor_pid" ]; then
        kill -INT "$monitor_pid" 2>/dev/null || true
        wait "$monitor_pid" 2>/dev/null || true
    fi
    if [ -n "$tegra_pid" ]; then
        kill -TERM "$tegra_pid" 2>/dev/null || true
        wait "$tegra_pid" 2>/dev/null || true
    fi
    if [ -n "$top_pid" ]; then
        kill -INT "$top_pid" 2>/dev/null || true
        wait "$top_pid" 2>/dev/null || true
    fi
    if pgrep -af 'ros2 bag record' | grep -F "$bag_name"; then
        echo 'recorderが残っています。制御sessionを維持して停止完了を待ちます。' >&2
        while pgrep -af 'ros2 bag record' | grep -F "$bag_name" >/dev/null; do sleep 1; done
    fi
    if [ -f "$bag_dir/metadata.yaml" ]; then
        ros2 bag info "$bag_dir" > "$run_dir/bag_info.txt" 2>&1
        echo "bag_complete=verified $bag_dir"
    else
        echo "metadata missing: $bag_dir" >&2
        return 4
    fi
}
trap finish_trial EXIT
trap 'exit 130' INT TERM

# VO初期化とtimestamp TF fusion成立を計測前に確認。jerkなし静止初期化設定を使用。
init_ok=false
for ((i=0; i<180; i++)); do
    if grep -qi 'successful initialization' "$run_dir/launch.log"; then init_ok=true; break; fi
    kill -0 "$launch_pid" 2>/dev/null || exit 4
    sleep 1
done
if [ "$init_ok" != true ]; then
    echo 'VO初期化未確認。明るさ・静止条件・jerk必要性をログで確認してください。' >&2
    exit 4
fi
initial_lines="$(wc -l < "$run_dir/launch.log")"
fusion_ok=false
for ((i=0; i<90; i++)); do
    count="$(tail -n +$((initial_lines+1)) "$run_dir/launch.log" | grep -Ec 'fusion=([1-9][0-9]*\.[0-9]+|0\.[0-9]*[1-9][0-9]*) Hz,.*drops\(rate/tf/queue/info\)=[0-9]+/0/[0-9]+/[0-9]+' || true)"
    if [ "$count" -ge 2 ]; then fusion_ok=true; break; fi
    kill -0 "$launch_pid" 2>/dev/null || exit 4
    sleep 1
done
[ "$fusion_ok" = true ] || { echo 'timestamp TF fusion未確認' >&2; exit 4; }
# Witの別検証フラグはこの比較で必ずOFF。設定の実効値も計測前に確認する。
timeout 20 ros2 param get /hwt905_imu_node timing_diagnostics > "$run_dir/wit_timing_parameter.txt"
grep -Eiq '(^|[[:space:]])false([[:space:]]|$)' "$run_dir/wit_timing_parameter.txt" || {
    echo 'Wit timing_diagnostics=falseを確認できません。試験中断。' >&2; exit 4;
}
timeout 20 ros2 param dump /depth_elevation_mapper --output-dir "$run_dir" \
    > "$run_dir/parameter_dump.log"
timeout 20 ros2 param get /depth_elevation_mapper depth_subscription_reliability \
    > "$run_dir/reliability_parameter.txt"
grep -Eq "String value is: $mapper_reliability$" "$run_dir/reliability_parameter.txt" || {
    echo 'reliabilityの実効parameterが指定条件と一致しません。試験中断。' >&2; exit 4;
}
timeout 20 ros2 param get /depth_elevation_mapper depth_subscription_queue_depth \
    > "$run_dir/history_depth_parameter.txt"
grep -Eq "Integer value is: $mapper_history_depth$" "$run_dir/history_depth_parameter.txt" || {
    echo 'history depthの実効parameterが指定条件と一致しません。試験中断。' >&2; exit 4;
}
sha256sum "$(ros2 pkg prefix pm_perception)/share/pm_perception/config/depth_elevation_mapper.yaml" \
    > "$run_dir/mapper_yaml_sha256.txt"
echo 'VO初期化・timestamp TF fusion確認。scheduler計測開始。'
start_probe() {
    probe_args=()
    if [ "$compare_reliable" = true ]; then probe_args+=(--compare-reliable); fi
    # 追加readerの影響を混ぜないよう、主A/Bとは別の補助試行としてのみ有効化する。
    python3 -c 'import os,signal,sys; signal.signal(signal.SIGINT,signal.SIG_DFL); os.execvp(sys.argv[1],sys.argv[1:])' \
        python3 "$task_workspace/src/pm_evaluation/tools/depth_delivery_probe.py" \
        --output "$run_dir/raw_probe.csv" "${probe_args[@]}" > "$run_dir/raw_probe.log" 2>&1 &
    probe_pid=$!
}
network_snapshot() {
    # 設定を変更せず、同じmapper生存中のUDP kernel drop累積値をphase境界で保存する。
    python3 -c 'import pathlib,time; print(time.monotonic_ns()); print(pathlib.Path("/proc/net/snmp").read_text()); print(pathlib.Path("/proc/net/udp").read_text()); print(pathlib.Path("/proc/net/udp6").read_text())' \
        > "$run_dir/network_$1.txt"
}
if [ "$probe" = true ]; then
    start_probe
    sleep 2
    kill -0 "$probe_pid"
fi
tegrastats --interval 1000 --logfile "$run_dir/tegrastats.log" &
tegra_pid=$!
# process別CPUも保存する。bag解析など別の高負荷processの混入を後から確認する。
top -b -d 1 -n "$duration" -w 240 > "$run_dir/top.log" &
top_pid=$!
ros2 run pm_perception mapper_executor_diagnostics monitor \
    --output "$run_dir/schedstat.csv" --duration-sec "$duration" --interval-sec 1.0 \
    --wait-timeout-sec 10 > "$run_dir/monitor.log" 2>&1 &
monitor_pid=$!
if [ "$probe" = toggle ]; then
    network_snapshot off_before
    sleep "$((duration / 3))"
    network_snapshot on_before
    start_probe
    sleep "$((duration / 3))"
    kill -0 "$probe_pid"
    kill -INT "$probe_pid"
    wait "$probe_pid"
    probe_pid=''
    network_snapshot off_after
fi
wait "$monitor_pid"
monitor_pid=''
if [ "$probe" = toggle ]; then network_snapshot end; fi
echo "計測完了。SIGINT停止とbag確定を待ちます。run_dir=$run_dir"
