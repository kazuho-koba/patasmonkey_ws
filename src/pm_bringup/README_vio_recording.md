# OpenVINS検証走行の記録

起動系と必須/任意bagを分離する新構成は[README_recording.md](README_recording.md)を参照。
本書の一括launchは互換用として維持している。

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

## 停止中の初期化と従来のjerk待ちの切替

このlaunchが使用する`pm_config/config/oak_d_s2/estimator_config1.yaml`では、
`try_zupt: true`で停止中の静的初期化＋ZUPT（ゼロ速度更新）を有効にする。
画像とIMUを約3秒以上取得し、静止条件を満たせば、手でjerkを加えずに初期化する。
起動時はセンサユニット・車体を静止させ、ログの`successful initialization`を
確認してから動かす。特徴点不足や振動が大きい場合は、初期化までの時間が延びる。

`zupt_only_at_beginning: true`のため、通常のVIO更新へ移行した後にはZUPTを
再投入しない。初期静止区間ではZUPTが通常の特徴更新を代替するため、
点群に新しい特徴が追加されない場合がある。初期化成功ログとodometryも確認する。
現在のZUPT受入れ判定は画像視差中心（`zupt_chi2_multipler: 0`、
`zupt_max_disparity: 0.5` pixel）なので、起動直後の低速移動を避ける。

従来の方式へ戻すには同じYAMLの`try_zupt: false`だけを変更する。
`zupt_only_at_beginning`はtrueのままでよい。`init_dyn_use: false`と
`init_imu_thresh: 0.2`は両方式で維持する。停止初期化では閾値がIMU変動の上限、
jerk待ちでは静止から励起へ移る判定値になる。閾値を0にしない。
切替操作はYAMLだけで行い、追加launch引数は使わない。
停止中にもROS出力するには、OpenVINS側の「成功したZUPTを公開可能な状態更新と
扱う」修正が必要。旧バイナリでは初期化成功ログが出ても通常の特徴更新まで出力しない。

導入時はFoxy環境で外部workspaceの`ov_msckf`、main workspaceの`pm_config`を
ビルドし、実行時にこの順でoverlayをsourceする。以後のYAML切替は`pm_config`を
再ビルドすればよい（symlink installで反映されていても実効設定を確認する）。
試行の`config_snapshot/`には、選択した方式のYAMLも保存される。

既存bagでの比較には`pm_evaluation/tools/diagnose_vio_bags.py`を使える。
以下はFoxyコンテナ内でOpenVINSとpm_evaluationのoverlayをsourceした後の例。
同じ入力・校正のまま`--initialization-mode jerk`と`static_zupt`を切り替える。

```bash
cd /workspaces/patasmonkey_ws
python3 src/pm_evaluation/tools/diagnose_vio_bags.py \
  --bag bags/rosbag2_2026_09_21-18_41_03_openvins_jerk_validation \
  --output src/pm_evaluation/results/openvins_initialization_ab \
  --replay-startup --replay-duration 60 --rate 0.5 \
  --initialization-mode static_zupt --replay-tag static_zupt
```

比較側は末尾を`--initialization-mode jerk --replay-tag jerk`に変更する。
再試行は既存結果を上書きしないよう、別の`--output`か`--replay-tag`を使う。
OpenVINSのconsole/state/timingログ、odometry CSV、実行引数は結果配下に残る。

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
# Wit IMUの周期診断

`pm_bag_global_localization.launch.py`に`wit_timing_diagnostics:=true`を付けると、
Witドライバの読み取り・変換・publish時間、起床遅延、期限超過を計測します。
既定はfalseです。poll_hzや周期制御の変更は行いません。
終了時に試行ディレクトリの`wit_timing.csv`へ保存します。
`wit_timing_max_samples`は既定60000件で、超過すると古い行を破棄します。
SIGKILL・電源断では保存されないため、SIGINTで正常終了させてください。

```bash
ros2 launch pm_bringup pm_bag_global_localization.launch.py use_vehicle_interface:=false use_teleop:=false wit_timing_diagnostics:=true
python3 ~/ros2_ws/src/hwt905_rs485_driver/tools/analyze_imu_timing.py ~/patasmonkey_ws/bags/<試行名>/wit_timing.csv
```

診断対応の`hwt905_rs485_driver`を先にビルド・sourceしてください。
詳細なCSV項目・制約は同パッケージのREADMEを参照してください。

起動直後の空白を反復検証する場合は、同パッケージの試験runnerを使えます。
既存service/launchを停止し、IMUの二重接続を避けてから実行してください。
以下は40秒を3回記録し、操縦・モータ系を無効にして比較します。
画像bagも保存するため空き容量を確認し、SSH制御接続は最後まで維持してください。

```bash
python3 ~/ros2_ws/src/hwt905_rs485_driver/tools/run_imu_startup_trials.py --mode bringup --output /tmp/wit_startup_bringup
```

出力先は未作成のディレクトリを指定します。各bagの`wit_timing.csv`と
`wit_timing.json`に起動節目・取得時間を残し、出力先の`comparison.json`へ
0〜5秒、5〜10秒、10〜20秒、20秒以降の周波数・長い間隔・エラーをまとめます。
時刻基準はPython node初期化開始です。センサ内部の測定時刻ではありません。
