#!/bin/bash
set -e

source /opt/ros/foxy/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash
source /home/nvidia/patasmonkey_ws/install/setup.bash

export ROS_DOMAIN_ID=0
export PYTHONUNBUFFERED=1

exec ros2 launch pm_bringup pm_bag_shibetsu.py
