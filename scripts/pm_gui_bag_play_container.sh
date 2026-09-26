#!/usr/bin/env bash
# GUI wrapperと同じcontainer/user/domain設定でrosbagを再生する。
set -Eeuo pipefail

if [[ $# -lt 1 ]]; then
    echo "使い方: $0 BAG_DIRECTORY [TOPIC ...]" >&2
    exit 2
fi

BAG_PATH="$1"
shift

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
PM_GUI_ROS_DOMAIN_ID="${PM_GUI_ROS_DOMAIN_ID:-}"

if ! docker container inspect "${PM_GUI_CONTAINER}" >/dev/null 2>&1; then
    echo "開発container '${PM_GUI_CONTAINER}' が起動していません。docker/foxy-dev/run.shで起動してください。" >&2
    exit 1
fi

exec docker exec \
    --user "${PM_GUI_USER}" \
    --env HOME="${PM_GUI_HOME}" \
    --env PM_GUI_ROS_WS="${PM_GUI_ROS_WS}" \
    --env PM_GUI_WS="${PM_GUI_WS}" \
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
        if [ "$#" -gt 1 ]; then
            exec ros2 bag play -s mcap "$1" --topics "${@:2}"
        fi
        exec ros2 bag play -s mcap "$1"
    ' bash "${BAG_PATH}" "$@"
