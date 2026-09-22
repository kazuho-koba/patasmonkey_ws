User prompt:
Frame [odom] does not existと言われてしまいます。
bagファイル再生時のコマンドは以下ですが、変更すべきですか？

root@kazuho-host:/workspaces/patasmonkey_ws# ros2 run pm_evaluation bag_clock_player     /workspaces/patasmonkey_ws/bags/rosbag2_2026_07_26-09_17_38     --rate 1.0     --topic /wheel/odometry     --topic /vio/odometry     --topic /wit/imu     --topic /tf_static     --topic /oak/depth/image_raw     --topic /oak/color/image_raw

Codex final response:
topic指定は変更不要です。そのコマンドで正しいです。

`odom`はbagに記録されたTFを再生しているのではなく、`terrain_mapping_latest_localization_replay.launch.py`が最新localizationで入力を再計算して初めて生成します。そのため、次の3 terminalで同じ`ROS_DOMAIN_ID`を設定してください。`ROS_DOMAIN_ID`が片方でも異なると、RVizは`odom`を見つけられません。

terminal 1（先に起動）:

```bash
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
source /workspaces/patasmonkey_ws/install/setup.bash
export ROS_DOMAIN_ID=91
ros2 launch pm_perception terrain_mapping_latest_localization_replay.launch.py
```

terminal 2（topic指定はそのまま）:

```bash
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
source /workspaces/patasmonkey_ws/install/setup.bash
export ROS_DOMAIN_ID=91
ros2 run pm_evaluation bag_clock_player \
  /workspaces/patasmonkey_ws/bags/rosbag2_2026_07_26-09_17_38 \
  --rate 1.0 \
  --topic /wheel/odometry \
  --topic /vio/odometry \
  --topic /wit/imu \
  --topic /tf_static \
  --topic /oak/depth/image_raw \
  --topic /oak/color/image_raw
```

terminal 3:

```bash
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
source /workspaces/patasmonkey_ws/install/setup.bash
export ROS_DOMAIN_ID=91
rviz2 -d "$(ros2 pkg prefix pm_perception)/share/pm_perception/rviz/terrain_mapping.rviz"
```

RVizはbag playerを起動してから、latest localizerが初期化を終えるまで数秒から十数秒は`odom`がない旨を表示し得ます。その間は待ってください。確認用には、同じdomainの別terminalで次を実行します。

```bash
ros2 topic echo /odometry/local --once
ros2 run tf2_ros tf2_echo odom base_link
```

`/odometry/local`が一度でも表示されれば、`odom -> base_link`もpublishされ、RVizのFixed Frame=`odom`が解決されます。もし60秒再生しても表示されなければ、terminal 1のlaunchログを共有してください。
