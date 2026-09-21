#!/usr/bin/env bash
# Run a ROS 2 launch file in its own session and stop every child reliably.
#
# This wrapper is intended for systemd services and non-interactive SSH jobs.
# A background shell can inherit SIGINT=ignore; sending SIGINT only to the
# ros2-launch parent in that state can leave its nodes orphaned under PID 1.

set -Eeuo pipefail

if (( $# < 2 )); then
    echo "Usage: $0 <package> <launch-file> [launch-argument ...]" >&2
    exit 64
fi

grace_seconds="${ROS_LAUNCH_SHUTDOWN_GRACE_SECONDS:-15}"
if ! [[ "$grace_seconds" =~ ^[1-9][0-9]*$ ]]; then
    echo "ROS_LAUNCH_SHUTDOWN_GRACE_SECONDS must be a positive integer" >&2
    exit 64
fi

launch_pid=""
launch_pgid=""

stop_launch_group() {
    if [[ -z "$launch_pid" ]] || ! kill -0 "$launch_pid" 2>/dev/null; then
        return
    fi

    # The launch command is a session/process-group leader created by setsid.
    # SIGINT lets ros2 launch terminate its managed children and finalize bag
    # metadata before escalating to SIGTERM if a process is stuck.
    kill -INT -- "-$launch_pgid" 2>/dev/null || true

    local deadline=$((SECONDS + grace_seconds))
    while kill -0 "$launch_pid" 2>/dev/null && (( SECONDS < deadline )); do
        sleep 1
    done

    if kill -0 "$launch_pid" 2>/dev/null; then
        echo "ROS launch did not exit after ${grace_seconds}s; sending SIGTERM to process group ${launch_pgid}" >&2
        kill -TERM -- "-$launch_pgid" 2>/dev/null || true
    fi
}

handle_signal() {
    local signal_name="$1"
    local exit_code="$2"
    trap - INT TERM HUP
    echo "Received ${signal_name}; stopping ROS launch process group" >&2
    stop_launch_group
    wait "$launch_pid" 2>/dev/null || true
    exit "$exit_code"
}

trap 'handle_signal INT 130' INT
trap 'handle_signal TERM 143' TERM
trap 'handle_signal HUP 129' HUP

# Reset ignored signal dispositions inherited from nohup/background shells,
# then exec ros2 so this PID is the process-group leader created by setsid.
setsid bash -c 'trap - INT TERM HUP; exec ros2 launch "$@"' bash "$@" &
launch_pid=$!
launch_pgid="$(ps -o pgid= -p "$launch_pid" | tr -d '[:space:]')"

if [[ "$launch_pgid" != "$launch_pid" ]]; then
    echo "Failed to create an isolated ROS launch process group" >&2
    kill -TERM "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
    exit 1
fi

wait "$launch_pid"
