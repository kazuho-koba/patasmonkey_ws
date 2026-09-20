# オドメトリ入力の切り分け

`compare_odometry_sources` は記録済み `/wheel/odometry`, `/wit/imu`,
`/vio/odometry` を5個の **実際のFoxy robot_localization EKF** へ再生する。
モータ状態からのホイール再計算、画像からのOpenVINS再計算は行わない。
入力生成側の状態を記録時のまま固定することで、local EKFに入る各ソースを比較する。

| 設定 | 使用する観測 | 元のlocal EKFからの変更 |
| --- | --- | --- |
| source_wheel.yaml | wheel vx, wz | 2D、旋回はホイールのみ、poseを重複融合しない |
| source_wheel_imu.yaml | wheel vx、IMU yaw, wz | 2D、初期yawのみrelative、roll/pitchとVIOを除外 |
| source_wheel_gyro.yaml | wheel vx、IMU wz | 2D、Witのorientationを除外し、z角速度だけを積分 |
| source_wheel_gyro_vio.yaml | wheel vx、IMU wz、VIO vx/vy | 2D、Wit orientationとwheel wzを除外、元のlocal EKFと同じくVIO twistを融合 |
| source_vio.yaml | VIO xyz, roll/pitch/yaw | 3D、VIO poseのみ、relative位置・姿勢 |

ベース設定は `pm_config/config/ekf_local_whl_imu_cam.yaml`。
50 Hz、sensor_timeout=0.1、入力の共分散、既定のプロセス雑音は維持する。
キューは再生時の欠落回避用に増やす。TF出力は無効、use_sim_timeを有効にする。
wheelのvyは使わない（記録値は0だが分散1e6であり、強い非横滑り制約ではない）。
第2条件の「旋回」は元の設定に合わせてyawとz角速度の両方を意味する。
ジャイロ積分だけの比較ではない。
VIO速度だけでは独立したyaw観測がないため、VIO条件ではposeを使う。
従って5条件とも元の融合EKFをそのまま再現したものではない。

## 実行

Foxyコンテナに `ros-foxy-robot-localization` とPython依存
（numpy, matplotlib, pyproj, yaml, requirements-analysis.txt）が必要。
この調査では不足していたrobot_localization 3.1.2をコンテナに導入した。

ホストからビルド（実行時のみPatasmonkey overlayをsource）:

```bash
docker exec --user 1000:1000 --env HOME=/tmp patasmonkey_foxy_dev bash -lc '
  source /opt/ros/foxy/setup.bash
  source /workspaces/ros2_ws/install/setup.bash
  cd /workspaces/patasmonkey_ws
  colcon build --packages-select pm_evaluation
'
docker exec --user 1000:1000 --env HOME=/tmp patasmonkey_foxy_dev bash -lc '
  source /opt/ros/foxy/setup.bash
  source /workspaces/ros2_ws/install/setup.bash
  source /workspaces/patasmonkey_ws/install/setup.bash
  cd /workspaces/patasmonkey_ws
  ros2 run pm_evaluation compare_odometry_sources \
    bags/rosbag2_2026_07_26-09_17_38 \
    --output src/pm_evaluation/results/rosbag2_2026_07_26-09_17_38 \
    --rate 1
'
```

実行スクリプト自身がEKFの起動・終了を管理するためlaunchは不要。
同名の結果CSVがある出力先への再計算は拒否する。別の出力ディレクトリを使う。
`--plot-only` は保存済みCSVから再描画する。
`--sources wheel_gyro` のように指定すると、必要な入力だけを読み、指定した
EKF条件だけを起動・描画する。VIOが破綻したbagのwheel+gyro評価などに使用する。
`--heading-offset-deg 0` 等で表示方位の感度を確認できる。
`--output-suffix gnss_offset` を併用すると既存画像・summaryを残して別名保存する。
`--no-basemap` はオフライン用。

ROS_DOMAIN_ID=87、ROS_LOCALHOST_ONLY=1を設定し、装置の実ノードから隔離する。
同じドメインで他の再生を並行実行しないこと。変更には `--domain-id` を使う。
記録時刻の順序・メッセージheader時刻を保存し、100 Hzの/clockで再生する。
画像を再生せず、静的TFはbagから集約する。動的TF、GNSS、記録済みEKFは入力しない。
終了後の外挿を比較に混ぜない。既存ソース全ての共通時間区間を使い、開始後1秒を空ける。
rate=1を基準とする。再生のコールバックタイミングによる微小な非決定性はある。

## 出力と解釈

- `comparison_map.png`: GNSS走行範囲で5条件を地図に重ねる。
- `comparison_full.png`: 逸脱した推定も含む全範囲。地図タイルは走行周辺のみ。
- `source_panels.png`: 条件別。wheel/VIOには入力の生poseも併記する。
- `input_signals.png`: 前進速度、旋回角速度、相対yaw。
- `*.csv`: header時刻の推定結果・入力・GNSS、地図整列済みメートル座標。
- `summary.json`: 共通区間、GNSSとの相違、入力比較、地図取得成否。
- `input_audit.json`: メッセージ数、frame、記録時刻とheaderの差。
- `source_*.yaml`, `*.log`, `run.json`: 使用設定、各EKFログ、再生条件。
- `basemap.npz`: 再描画用の取得済みOSM背景。

共通時刻のGNSS位置とIMU yawを使って各軌跡の初期位置・方位だけを整列する。
元のplot_bag_trajectoriesと同じ **IMU yaw +90度** を既定とするが、これは
表示上の仮定であり、この調査で校正が検証されたことを意味しない。
全軌跡を最小二乗でGNSSに合わせたり、スケールを補正したりしない。
初期座標は各poseのyawが位置座標と整合する前提。VIOアダプタの姿勢と位置の
変換に不整合があると、初期方位合わせ自体にも影響する。
距離はGNSS付近の正距方位図法のメートルで計算し、描画時だけEPSG:3857へ変換する。
旧スクリプトのように地上メートルをそのままWeb Mercatorへ足さない。

GNSSはアンテナ位置、odomはbase_link原点。アンテナの約7 cmの水平オフセットは
補正していない。GNSSの品質・初期方位仮定の影響があるため、RMSEは真値誤差ではなく
**GNSSとの不一致**。入力信号比較はheader時刻で補間し、遅延補正はしていない。
記録時刻との差はセンサ処理・輸送・記録の合計であり、時計ずれだけとは断定できない。
OSM取得に失敗した場合は背景なしで保存し、summary.jsonにエラーを記録する。

## 初期heading offsetの検証

`tools/analyze_heading_offset.py <results>` は、wheel_gyroのローカル速度・位置と
GNSSのENU速度 `/fix_velocity`・位置差を比較し、plot時に公開済みWit yawへ
加える角度を推定する。速度閾値・GNSS速度共分散閾値、5～30秒位置差による
感度もJSONへ保存する。

現行のHWT905ドライバはセンサyawから90度を引き、URDFのwit_imu_link→base_link
yawは0度である。一方、plotの+90度はドライバ処理を相殺する。したがって
`+90度`は地理的な校正値とみなさない。正しくROS ENUへ変換済みのIMU topicなら
plot側の理論offsetは0度である。磁気yawを真北へ合わせる場合は、さらに場所・日時の
磁気偏角と車載磁気外乱の校正が必要になる。
