# 記録VIOと車輪・IMUによるlocal EKFの比較

`plot_vio_wheel_imu_comparison`は、事前に実行した車輪vx＋Wit wzのみの
`robot_localization`出力、bagの`/ov_msckf/odomimu`、`/odometry/local`、
`/fix`を地図へ重ねます。横方向を分担する既定の2D EKFの設定からVIO入力を
無効化したfilterを使用します。加速度・Wit orientation・VIOは入力しません。

入力は対象bag群のディレクトリ、各bagと同名のサブディレクトリにある
`replayed_bag`（`/comparison/wheel_imu`を記録）、既存OSMキャッシュ、
`plot_recorded_local_global`で作成したsummaryと各bag内の
`recorded_local_enu.csv`です。再計算には`bag_clock_player`で
`/wheel/odometry`、`/wit/imu`、`/tf_static`を等倍再生し、
`use_sim_time`を有効にしたEKFの出力を記録してください。

```bash
ros2 run pm_evaluation plot_vio_wheel_imu_comparison BAG_ROOT \
  --replay-root REPLAY_ROOT --basemap BASEMAP.npz \
  --reference-summary RECORDED_SUMMARY.json
```

比較図と時系列CSVは各bagディレクトリへ、概要JSONは再計算先へ保存します。
出力ファイルは同名ファイルを上書きします。VIOはOpenVINSのROS quaternionを
そのまま使用し、速度の軸補正に基づく仮の取付回転 `(x,y,z)=(-z,-y,-x)` と
記録済みstatic TFの平行移動でIMU位置を車体位置へ変換します。この仮定は小さな
実機動作との整合を確認しましたが、厳密な取付校正ではありません。描画設定は
実機adapterの姿勢反転設定とは独立で、走行中の推定は変更しません。
開始時刻の記録local位置・yawを基準にします。開始前の誤差や基準自体の方位誤差は
引き継ぎます。VIO軌跡はゲート後のEKF入力ではありません。ゲート診断は状態ごとの
診断メッセージ件数であり、VIOデータの通過件数ではありません。

再計算EKFはyaw=0で起動し、最初の共通時刻に記録localの位置・yawへ平面回転と
平行移動を適用します。これは水平フィルタの回転対称性を用いた可視化上の座標変換で、
GNSSへの軌跡フィットではありません。元のmap→odomは共通基準時刻で固定します。
GNSSと各系列の距離は共通時刻の水平差であり、GNSSも誤差を持つので真値誤差とは
限りません。VIO開始時刻が異なるため系列間の中央値も同一時間区間の比較ではありません。

## 方位オフセットなしの比較

上記コマンドに `--no-heading-offset` を追加すると、通常出力に上書きせず、
`*_no_heading_offset.png`、CSV、および `summary_no_heading_offset.json` を
保存します。localは前回適用した基準時刻のmap→odomのyawを取り消し、
車輪＋IMU再計算とOpenVINSは開始方位への回転を行いません。どの系列も
平行移動で開始位置を合わせ、VIOはその開始時刻のlocal位置に合わせます。
カメラIMUからbase_linkへの取り付けTFは、方位補正ではなく物理的な
センサ取付位置の変換なので適用したままです。

この図は、各ソースの異なる座標系のx/yを東/北と仮定して重ねた比較用の図です。
正式な地図座標変換ではないため、GNSSに近いかどうかだけで精度を判断できません。
方位を含む姿勢の初期化やドライバ設定そのものを変更してEKFを再計算したものでもありません。

## 磁気方位オフセットをゼロとした概算

`--remove-magnetic-offset` はbagの `heading_initializer_node` の実効設定から
`offset_deg` を読み、`yaw = π/2 − (磁気方位 + offset)` に基づき、各系列を
共通基準点の周りに `offset_deg` だけ回転します。地図との座標系変換は通常どおり
適用します。元の方位オフセットが−12.9701°なら、ゼロとした場合のENU yawは
−12.9701°変化します。出力には `_magnetic_offset_zero_approx` を付与します。

これはbag開始以前の初期位置とEKFの内部状態、VIOゲート入出力を完全には
再現できない場合の**概算**です。記録開始時点を回転の支点とするため、
実際の初期化点から走行を始めた結果とは異なり得ます。既存のEKF結果を回転
しており、共分散・異常判定・再投入などを新しい初期方位で再実行したものでは
ありません。bag開始前の欠落は計算で復元できません。

## bagと解析結果を別ディレクトリに置く場合

`--output-root` に解析結果の親ディレクトリを指定すると、各bag名の
サブディレクトリにある `recorded_local_enu.csv` を読み、生成画像とCSVも
そこへ保存します。元bagは引き続き `BAG_ROOT` から読みます。たとえば
`--output-root /workspaces/patasmonkey_ws/src/pm_evaluation/results/rosbag2_2026_10_30-homearound`
を指定します。`plot_recorded_local_global` と `plot_recorded_height` にも
同名オプションがあり、先にlocalのCSVを同じ保存先へ生成できます。

従来の `plot_bag_trajectories` は `/vio/odometry` を描画する別のCLIであり、
上記のOpenVINS生姿勢・仮取付回転によるVIO単独軌跡とは入力と整列方法が異なります。
VIO単独軌跡の比較には本CLIを使用してください。
