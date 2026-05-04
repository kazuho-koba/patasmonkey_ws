#!/usr/bin/env bash
set -e

IMAGE_NAME="patasmonkey:foxy-dev"
CONTAINER_NAME="patasmonkey_foxy_dev"

# この run.sh の場所からリポジトリルートを推定
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST_WS="$(cd "${SCRIPT_DIR}/../.." && pwd)"
CONTAINER_WS="/workspaces/patasmonkey_ws"

# GUI 使用許可
xhost +local:docker
xhost +local:root
xhost +local:$(whoami)


# --rmをつけるとコンテナ停止時に破棄
docker run -it --rm \
  --name "${CONTAINER_NAME}" \
  --network host \
  --ipc host \
  --pid host \
  --gpus all \
  --env NVIDIA_VISIBLE_DEVICES=all \
  --env NVIDIA_DRIVER_CAPABILITIES=all \
  --env DISPLAY="${DISPLAY}" \
  --env QT_X11_NO_MITSHM=1 \
  --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  --env RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume "${HOST_WS}:${CONTAINER_WS}:rw" \
  --volume /dev:/dev \
  --group-add dialout \
  --group-add video \
  --privileged \
  --workdir "${CONTAINER_WS}" \
  "${IMAGE_NAME}"