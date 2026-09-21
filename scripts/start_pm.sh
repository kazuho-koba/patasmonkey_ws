#!/bin/bash
set -e

source /opt/ros/foxy/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash
source /home/nvidia/patasmonkey_ws/install/setup.bash

export ROS_DOMAIN_ID=0
export PYTHONUNBUFFERED=1

# Keep ROS launch and every node in one dedicated process group.  This makes
# `systemctl stop` (and non-interactive SSH shutdown) terminate the whole
# launch tree instead of leaving sensor/VIO nodes orphaned under PID 1.
exec "$(dirname "$0")/run_ros_launch.sh" \
  pm_bringup pm_bag_shibetsu.launch.py
