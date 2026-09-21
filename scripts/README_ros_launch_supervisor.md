# ROS launch supervisor

Use `run_ros_launch.sh` for service and background execution of ROS 2 launch
files. It creates a dedicated session/process group, resets signal dispositions
that may have been inherited from `nohup` or a background shell, and stops the
whole group with SIGINT before escalating to SIGTERM after 15 seconds.

`start_pm.sh` uses it automatically. For an explicit global-localization run:

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/patasmonkey_ws/install/setup.bash
~/patasmonkey_ws/scripts/run_ros_launch.sh \
  pm_bringup pm_bag_global_localization.launch.py
```

For systemd, retain the script as `ExecStart` and configure
`KillMode=control-group`; systemd then also catches any process that cannot be
managed by ROS launch itself.
