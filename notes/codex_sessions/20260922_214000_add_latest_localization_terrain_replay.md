User prompt:
7月bagのオドメトリだけ最新の計算方法に直して可視化できないですか？

Codex final response:
できます。7月bagの画像・sensor・static TFはそのまま再生し、`/tf`にある過去の`odom→base_link`だけを再生せず、現在の`separated_offroad`相当のlocalizationで新しく計算する可視化経路を追加しました。

再計算する内容は、現行のhorizontal EKF（wheel + WIT gyro z + gate済みVIO twist）、VIO vertical gate、attitude/height observer（WIT roll/pitch + gate済みVIO z）、local odometry composerです。composerが再計算済みの`/odometry/local`と唯一の`odom→base_link` TFをpublishし、terrain mapperはそのTFを使います。

追加したものは次の2点です。

- `pm_perception/launch/terrain_mapping_latest_localization_replay.launch.py`
- `pm_evaluation/cli/bag_clock_player.py`のRGB-D streaming対応。画像全体をメモリに積まずMCAPから順次再生します。

実行は、3端末すべてで同じROS domainを指定してください。

terminal 1:

```bash
docker exec -it --user 1000:1000 --env HOME=/tmp patasmonkey_foxy_dev bash
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
source /workspaces/patasmonkey_ws/install/setup.bash
export ROS_DOMAIN_ID=91

ros2 launch pm_perception terrain_mapping_latest_localization_replay.launch.py
```

terminal 2:

```bash
docker exec -it --user 1000:1000 --env HOME=/tmp patasmonkey_foxy_dev bash
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
docker exec -it --user 1000:1000 --env HOME=/tmp patasmonkey_foxy_dev bash
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
source /workspaces/patasmonkey_ws/install/setup.bash
export ROS_DOMAIN_ID=91

rviz2 -d "$(ros2 pkg prefix pm_perception)/share/pm_perception/rviz/terrain_mapping.rviz"
```

重要なのはterminal 2の入力topicに`/tf`を入れないことです。旧TFと再計算TFが競合しません。

検証済みです。7月bagを独立ROS domainで50秒再生し、最新localizer由来TFでterrain mapperがdepth→grid処理まで到達しました。代表frame処理は約1–1.4 ms、`tf_drops=0`でした。再生をtimeoutで止めた境界の最後のframeだけはfuture extrapolationでdropしましたが、通常の最後までの再生では問題になりません。

`pm_evaluation`全体には既存の多数のflake8違反がありますが、今回変更したbag playerと新launchはstyle検査を通過し、両packageのFoxy buildも成功しています。
