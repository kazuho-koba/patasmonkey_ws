# OpenVINS検証走行の記録

`pm_bag_global_localization.launch.py` は、通常のセンサ・自己位置推定topicに
加えて、OpenVINSの再現・原因調査に必要な診断情報を記録する。

## 実行

```bash
ros2 launch pm_bringup pm_bag_global_localization.launch.py
```

既定では `~/patasmonkey_ws/bags/rosbag2_YYYY_MM_DD-HH_MM_SS/` が作られる。
試行名を明示する場合は次のように指定する。

```bash
ros2 launch pm_bringup pm_bag_global_localization.launch.py \
  bag_name:=rosbag2_test_course_a_01
```

既存ディレクトリ名を指定すると `ros2 bag record` が安全のため失敗する。

## 同一試行に保存されるOpenVINS情報

bagディレクトリ内の `openvins/` に以下を保存する。

- `state_estimate.txt`: pose、velocity、IMU bias、オンライン校正値を含む全状態
- `state_deviation.txt`: 全状態の標準偏差
- `timing.txt`: OpenVINS更新処理の実行時間
- `console.log`: `/rosout` には載らない初期化・更新ログを含む標準出力
- `ov_msckf__run_subscribe_msckf.yaml`: ROSパラメータの実効値
- `trial_manifest.json`: hostname、Git revision、config hash
- `*_git_status.txt`, `*_git_diff.patch`: 未コミット変更を含む実行時ソース状態
- `config_snapshot/`: estimator、IMU、camera-IMU校正YAMLのスナップショット

bagには次のOpenVINS診断topicも追加される。

- `/ov_msckf/trackhist`: OpenVINSが実際に追跡している特徴の可視化画像
- `/ov_msckf/points_msckf`: MSCKF更新で良好と判定された3D特徴
- `/ov_msckf/points_slam`: SLAM特徴。`max_slam: 0` の設定では通常空

`trackhist` と特徴点群はsubscriberがある場合だけ生成されるため、単にpublisherが
存在するだけでは保存されない。launchの明示的なrecord対象に含めることで生成を
有効化している。

OAK-D S2については、`diagnostic_msgs/DiagnosticArray` の安定したkey schemaで
DepthAI固有の時刻・sequence情報も保存する。

- `/oak/diagnostics/left_frame`, `right_frame`: 左右それぞれのdevice時刻、
  sequence、欠落数、露光時間、ISO感度
- `/oak/diagnostics/color_frame`, `depth_frame`: RGB-D出力の同等metadata
- `/oak/diagnostics/imu_packet`: accel/gyro個別device時刻、sequence、時刻差、
  batch内位置、欠落数
- `/oak/diagnostics/device_info`: MX ID、実際のIMU種別、firmware、USB速度、
  DepthAI version

## 負荷と限界

診断を優先する検証用launchなので、通常走行よりディスク帯域とOpenVINSの描画負荷が
増える。特に `trackhist` が主要な追加容量となる。

DepthAI 2.30の画像APIは露光時間とISO感度を提供するが、独立したアナログgain値は
提供しない。このため診断messageでは `sensitivity_iso` を保存する。device timestampは
OAK起動後のmonotonic durationであり、Unix時刻ではない。ROS header stamp、DepthAIの
host同期済みtimestamp、host受信時刻も併記し、時計変換と転送遅延を区別できるようにする。
