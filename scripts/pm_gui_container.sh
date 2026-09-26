#!/usr/bin/env bash
# 起動済み開発containerのROS overlayを読み込みpm_guiを起動する。
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ローカル環境に応じて環境ファイルからcontainer名・mount先等を上書きする。
CONFIG_FILE="${PM_GUI_ENV_FILE:-${HOME}/.config/patasmonkey/pm_gui.env}"
if [[ -f "${CONFIG_FILE}" ]]; then
    # shellcheck disable=SC1090
    source "${CONFIG_FILE}"
fi

PM_GUI_CONTAINER="${PM_GUI_CONTAINER:-patasmonkey_foxy_dev}"
PM_GUI_USER="${PM_GUI_USER:-1000:1000}"
PM_GUI_HOME="${PM_GUI_HOME:-/home/developer}"
PM_GUI_ROS_WS="${PM_GUI_ROS_WS:-/workspaces/ros2_ws}"
PM_GUI_WS="${PM_GUI_WS:-/workspaces/patasmonkey_ws}"
PM_GUI_CONFIG="${PM_GUI_CONFIG:-}"
PM_GUI_ROS_DOMAIN_ID="${PM_GUI_ROS_DOMAIN_ID:-}"

if ! docker container inspect "${PM_GUI_CONTAINER}" >/dev/null 2>&1; then
    echo "開発container '${PM_GUI_CONTAINER}' が起動していません。docker/foxy-dev/run.shで起動してください。" >&2
    exit 1
fi

exec docker exec \
    --user "${PM_GUI_USER}" \
    --env HOME="${PM_GUI_HOME}" \
    --env DISPLAY="${DISPLAY:-}" \
    --env QT_X11_NO_MITSHM=1 \
    --env PM_GUI_ROS_WS="${PM_GUI_ROS_WS}" \
    --env PM_GUI_WS="${PM_GUI_WS}" \
    --env PM_GUI_CONFIG="${PM_GUI_CONFIG}" \
    --env PM_GUI_ROS_DOMAIN_ID="${PM_GUI_ROS_DOMAIN_ID}" \
    "${PM_GUI_CONTAINER}" bash -lc '
        if [ ! -w "${HOME}" ]; then
            export HOME=/tmp
        fi
        if [ -n "${PM_GUI_ROS_DOMAIN_ID}" ]; then
            export ROS_DOMAIN_ID="${PM_GUI_ROS_DOMAIN_ID}"
        fi
        source /opt/ros/foxy/setup.bash
        if [ -f "${PM_GUI_ROS_WS}/install/setup.bash" ]; then
            source "${PM_GUI_ROS_WS}/install/setup.bash"
        fi
        source "${PM_GUI_WS}/install/setup.bash"
        if [ -n "${PM_GUI_CONFIG}" ]; then
            exec ros2 launch pm_gui operator_console.launch.py config:="${PM_GUI_CONFIG}"
        fi
        exec ros2 launch pm_gui operator_console.launch.py
    '
