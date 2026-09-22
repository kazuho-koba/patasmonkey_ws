User prompt

ありがとうございます。ではその方向で、実装と検証を進めてもらえますか？

確認は進めてもらって構わないです。追加指示として、この初期化のやり方を使った場合のオドメトリとGNSS信号と地図画像の重畳プロットを作って欲しいです。

Codex final response

ステップ3の初期heading方式を実装し、7月bagで初期化処理とGNSS・OpenStreetMap重畳まで検証しました。

実装内容

- [heading_initializer_node.py](/home/kazuho/Projects/patasmonkey_ws/src/pm_localization/pm_localization/heading_initializer_node.py)を追加しました。
  - `/wit/imu`を受け、`yaw_correction_radians`だけyawを補正した`/wit/imu/heading_calibrated`を出力します。roll/pitch、角速度、加速度は変更しません。
  - `/wheel/odometry`の`|vx| <= 0.03 m/s`を確認している間だけ、100サンプル（Wit 100 Hzなら約1秒）の円平均を取ります。
  - 円標準偏差が0.10 rad以下の場合に限り、Foxyの`/set_pose`サービスで水平local EKFのyawを一度だけ初期化します。その後、local yawはWit `wz`だけで伝播します。
  - 初期化に失敗・タイムアウトした場合は、自動的な再初期化をしません。
  - 初期yawと有効な`/fix`が揃った後、Foxyの`/datum`サービスを呼び、navsat_transformのdatumを確立します。

- [heading_initializer.yaml](/home/kazuho/Projects/patasmonkey_ws/src/pm_config/config/heading_initializer.yaml)に校正値・閾値を集約しました。現時点の`yaw_correction_radians`は意図的に`0.0`です。これは未校正値を運用値として採用したものではなく、校正試験で求める固定補正値の差し込み口です。

- `separated_offroad`時だけ、navsat_transformは[navsat_transform_heading_initialized.yaml](/home/kazuho/Projects/patasmonkey_ws/src/pm_config/config/navsat_transform_heading_initialized.yaml)を使用します。この設定は`wait_for_datum: true`です。従って静止heading初期化に失敗した場合、`/odometry/gps`は意図的に出力されず、未検証のheadingでglobal EKFを拘束しません。既存`legacy`モードのnavsat設定は維持しています。

- 両方のlocalization launchで、`separated_offroad`時だけnavsat_transformのIMU入力を`/wit/imu/heading_calibrated`へ切り替えました。bag launchはこの補正済みIMUも記録します。

- [plot_initialized_odometry_overlay.py](/home/kazuho/Projects/patasmonkey_ws/src/pm_evaluation/pm_evaluation/cli/plot_initialized_odometry_overlay.py)を追加しました。既存`plot_bag_trajectories.py`のGNSS局所座標化とOpenStreetMap描画を再利用し、再計算CSVをGNSSへ「平行移動だけ」で重畳します。描画時のyaw回転・`pi/2`補正は一切行いません。従って、校正値が間違っていればプロット上でそのまま方位ずれとして現れます。

検証結果

7月bagの再生では、停止中サンプルから初期yaw `2.819 rad`（約161.5度）、円標準偏差`0.005 rad`（約0.29度）を得ました。`/set_pose`の一回呼出し後、horizontal EKFのyawが0から約2.819 radへ切り替わることをCSVで確認しました。

ただし補正値をまだ0としたため、これは「生Wit yawで初期化した比較」です。GNSS重畳の誤差はRMSE 36.01 m、中央値14.83 m、最大86.61 mでした。これはGNSS自体にNO FIX/FLOATが含まれることに加え、Witの磁気偏角・取付角・磁気外乱をまだ校正していないためで、運用精度の達成を意味しません。むしろ、補正なしの磁気yawを採用できないことを定量的に示す基準結果です。

生成物はこちらです。

- [GNSS・OSM重畳プロット](/home/kazuho/Projects/patasmonkey_ws/src/pm_evaluation/results/heading_initialization/rosbag2_2026_07_26-09_17_38_rate10_final/gnss_osm_overlay/initialized_odometry_gnss_osm_overlay.png)
- [重畳誤差のJSON](/home/kazuho/Projects/patasmonkey_ws/src/pm_evaluation/results/heading_initialization/rosbag2_2026_07_26-09_17_38_rate10_final/gnss_osm_overlay/initialized_odometry_gnss_overlay_summary.json)
- [初期化付きodometry CSV・再生MCAP・ログ](/home/kazuho/Projects/patasmonkey_ws/src/pm_evaluation/results/heading_initialization/rosbag2_2026_07_26-09_17_38_rate10_final)

実機では、車両を停止させたまま次を起動し、`heading_initializer_node`の「Seeded local EKF yaw once」と「Sent navsat datum」の両ログを確認してから走行します。

```bash
ros2 launch pm_bringup pm_bag_global_localization.launch.py \
  localization_mode:=separated_offroad
```

次に必要なのは、搭載状態・電源/ODrive状態を含めたWit地磁気校正と、直進GNSS courseを使った`yaw_correction_radians`の符号・値の実測です。この値が確定するまでは、今回の0 rad設定で地球基準headingを運用に使うべきではありません。

`pm_config`、`pm_localization`、`pm_bringup`、`pm_evaluation`のビルド、両launchの非起動構文確認、Python構文確認、`git diff --check`は成功しています。Foxy実装上のサービス名も隔離ノードで`/set_pose`と`/datum`を確認済みです。Jetson実機への反映・実走はまだ行っていません。
