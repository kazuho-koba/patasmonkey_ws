User prompt: 開発PC側でPhase 3の実装をお願いします。
その後、改めてどのような実装を行ったのか、教えてください。
また任意のbagファイルに対して、実装された結果どのようなことができるようになったのかを示す可視化の手順を教えてください（手順の実行自体はこちらで実施します）

Codex final response: Phase 3を開発PC側に実装しました。Stage 2のdepth→rolling 2.5D grid融合とlocalizationには手を入れず、融合済みの`relative_elevation`から軽量な局所terrain cueを追加しています。

主な変更ファイルは`src/pm_perception/pm_perception/terrain_features.py`、`depth_elevation_mapper_node.py`、`config/depth_elevation_mapper.yaml`、`rviz/terrain_mapping.rviz`、`README.md`です。加えて、`pm_evaluation`のbag playerへCameraInfo再生対応、`pm_bringup`の記録topicへStage 3の4 topicを追加し、synthetic terrain向けテストも追加しました。

追加したもの:

- `/depth_elevation_mapper/slope_debug` — central differenceによる局所勾配（0〜30°を黒〜白）。
- `/depth_elevation_mapper/roughness_debug` — freshな3×3近傍の高さ標準偏差（0〜0.08 m）。
- `/depth_elevation_mapper/step_height_debug` — freshな3×3近傍のmax-min高さ（0〜0.20 m）。
- `/depth_elevation_mapper/terrain_hazard_debug` — slope / roughness / step / obstacle heightを各暫定閾値で正規化した最大値（0〜1）。これは可視化用の暫定cueであり、Nav2 cost・走行禁止判定・negative obstacle検出ではありません。

特徴量はdepthフレームごとには計算せず、既存の`debug_publish_rate`（既定2 Hz）でのみ120×120程度のNumPy配列を処理します。PointCloud2は通常どおり不要です。古いセルや近傍観測が不足するセルはunknownのままにするため、「depthが無い」ことを穴・段差と誤認しません。

主な追加parameterは次です。

- `publish_stage3_debug_layers: true`
- `feature_max_observation_age: 3.0` s
- `feature_neighborhood_radius_cells: 1`（3×3）
- `feature_min_neighbors: 5`
- `hazard_slope_limit_deg: 20.0`
- `hazard_roughness_limit: 0.03` m
- `hazard_step_limit: 0.10` m
- `hazard_obstacle_height_limit: 0.20` m

RViz設定に4つのMap displayを追加しました。初期状態では無効なので、従来の表示への負荷は増えません。また`pm_bag_global_localization.launch.py`では4 topicを今後の計測bagへ記録します。`bag_clock_player`は`/oak/depth/camera_info`も再生できるようにしました。

任意bagを最新localizationで再計算して確認するには、対応するbagが`/wheel/odometry`、`/vio/odometry`、`/wit/imu`、`/tf_static`、`/oak/depth/image_raw`を含む前提で、次の3端末を使います。新しいbagでCameraInfoもある場合は第2端末のCameraInfo行を含め、古いbagではその行だけ外してください。記録済み`/tf`は再生しません。

```bash
# terminal 1
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
source /workspaces/patasmonkey_ws/install/setup.bash
export ROS_DOMAIN_ID=91
ros2 launch pm_perception terrain_mapping_latest_localization_replay.launch.py
```

```bash
# terminal 2: <bag_directory> を対象bagへ置換
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
source /workspaces/patasmonkey_ws/install/setup.bash
export ROS_DOMAIN_ID=91
ros2 run pm_evaluation bag_clock_player <bag_directory> --rate 1.0 \
  --topic /wheel/odometry --topic /vio/odometry --topic /wit/imu \
  --topic /tf_static --topic /oak/depth/image_raw \
  --topic /oak/depth/camera_info --topic /oak/color/image_raw
```

```bash
# terminal 3
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
source /workspaces/patasmonkey_ws/install/setup.bash
export ROS_DOMAIN_ID=91
rviz2 -d "$(ros2 pkg prefix pm_perception)/share/pm_perception/rviz/terrain_mapping.rviz"
```

RVizのDisplaysで`Slope (Stage 3)`、`Roughness (Stage 3)`、`Step height (Stage 3)`、`Preliminary terrain hazard (Stage 3)`を一つずつ有効化してください。Fixed Frame=`odom`、視点のTarget Frame=`base_link`のため、地図はodom上に残り、カメラだけがロボットを追従します。`Observation age (Stage 2)`を併用すると、FOV外の地面がmapに残ることと、3秒を超えるとStage 3表示だけunknownになることを確認できます。

検証結果: Foxyコンテナで`colcon build --packages-select pm_evaluation pm_perception`および`colcon build --packages-select pm_bringup`が成功し、`pm_perception`のテストは`9 passed`でした。

変更内容の詳細とコマンドは`src/pm_perception/README.md`にも追記済みです。
