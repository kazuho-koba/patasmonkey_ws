User prompt:
ありがとうございます、ではStage2を進めてもらえますでしょうか？
進めたら、内容についてもう少し詳細な解説もお願いしたいです。

検証をする際、オドメトリはbagファイルのものをそのまま使うのでなく、最新のやり方で計算し直すようにしてほしいです。

すみません、前のrvizやros launchが生きていたので全て止めました。作業を続けてください。

Codex final response:
Stage 2を実装しました。Stage 1の`odom`基準2.5D gridを残したまま、局所地面基準の高さ・観測品質/経過時間・最小限のground/obstacle分離を加えています。localization側は変更していません。

主な追加内容:

- `relative_elevation` layer: 各depth撮像時刻で`camera_z - 0.40 m`を期待地面高さとし、odom-z elevationとは別に正規化した高さを融合します。平坦路なら概ね0 mです。静的TFのbase_link→camera 0.28 mは変えません。
- 距離依存重み: `variance = measurement_variance + depth_variance_per_meter_sq × range²` とし、遠いdepthほど融合への寄与を下げます。セル内では保守的に最大分散を採用します。
- stale観測: `observation_decay_time`（既定8 s）で融合重みとobstacle confidenceを減衰させます。rawの観測数と最終観測時刻は残るため、古さを確認できます。
- obstacle候補: 同じXYセルの最高点とground候補（最低点）の差が0.20 m以上なら、obstacle height/confidenceを蓄積します。semantic認識や完全なground分類ではありません。
- debug topics: `relative_elevation_debug`、`elevation_variance_debug`、`observation_count_debug`、`observation_age_debug`、`obstacle_height_debug`を追加しました。すべて可視化用OccupancyGridでNav2 costではありません。RViz設定には追加済みで、初期状態は無効です。

重要な解釈:

`elevation_debug`は絶対的なodom-zであり、odomのz変動も見えます。`relative_elevation_debug`は撮像時のカメラ高を差し引くため、共通モードのzずれの影響を小さくします。ただし、これは長い斜面を一貫した世界高さとして表す完成版地図ではなく、まず平坦路での健全性を見る診断レイヤです。0.40 mは暫定値なので、実測後には`nominal_camera_height_above_ground`だけを更新します。

検証は、記録済み`/tf`・odomを使わず、最新のseparated_offroad相当localizationを再計算する既存launchを使います。terminal 1:

```bash
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
source /workspaces/patasmonkey_ws/install/setup.bash
export ROS_DOMAIN_ID=91
ros2 launch pm_perception terrain_mapping_latest_localization_replay.launch.py
```

terminal 2:

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

この再生では`/tf`を再生しません。current localizerが唯一の`odom -> base_link` TFをpublishします。RViz左側のDisplaysからStage 2 mapを一つずつ有効にしてください。特に、平坦区間でrelative elevationが大きく揺れないこと、接近後にFOV外となった地面がobservation ageとしてwindow内に残ること、岩/草でobstacle layerが過剰反応しないことを見ます。

変更ファイルは、主に`src/pm_perception/pm_perception/rolling_elevation_grid.py`、`depth_elevation_mapper_node.py`、`config/depth_elevation_mapper.yaml`、`rviz/terrain_mapping.rviz`、`README.md`、およびgrid単体テストです。

確認結果:

- Python unit test: 6 passed
- `ament_flake8`（pm_perception対象）: pass
- Foxy containerで`colcon build --packages-select pm_perception`: pass
- 7月bagを独立ROS domainで60秒再生: 最新localization経路でTF drop 0。約5,100〜5,500点/frameを約530〜580セルへ融合、平均約2.8 ms/frame（最大15.5 ms）でした。

次のStage 3は、局所平面を使ったground候補の堅牢化、vertical bands/negative-obstacle cue、slope・roughness・stepの計算、そして軽量な正式map messageの順が適切です。その後に初めてNav2 costへ変換します。
