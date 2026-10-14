# OpenVINS検証走行の記録

## 周期・配送遅延のA/B診断

`wit_recover_small_overrun:=true`（既定）はWitの小さなdeadline超過を回復します。
従来方式は小さな遅延でも次回の基準時刻を後ろへずらすため、平均取得周波数が下がります。
変更後は1周期未満の遅延だけ次周期で回復し、1周期以上の遅延は再同期します。
Jetsonのcore＋mission bag試験では、定常40秒の実取得が98.90 Hzから99.85 Hzに
改善しました。ただし非リアルタイムOSなので、常に厳密な100 Hzや無欠測を保証しません。
`wit_recover_small_overrun:=false`で従来方式へ戻せます。`wit_timing_diagnostics:=true`で
取得周期・read・publishを比較し、起動直後と定常区間を別集計してください。

`oak_timing_diagnostics:=true`でOAKのpoll・topic別publish所要時間とRGB-Dの
取得batch内省略数を、同じ試行ディレクトリの`oak_publish_timing.csv`へ正常停止時に
保存します。pm_coreでは`~/.ros/pm_core_sessions/<bag_name>`、旧bag一括launchでは
bagディレクトリが保存先です。計測は既定OFFで、topic/QoSと最新RGB-D選択は変更しません。
終了時保存に追加SIGINTを重ねないでください。時間計測自体の負荷を考慮し、
A/Bとも同じ診断設定を使用してください。

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

起動時のparameter dumpは既定で`--no-daemon`を使用し、一回のsnapshotのための
常駐ROS 2 daemon生成を避けます。比較時のみ
`openvins_parameter_dump_no_daemon:=false`で従来方式へ戻せます。
これは外部GUI等が別途生成するdaemonを停止する機能ではありません。

OpenVINSのIMU購読欠測を調べる場合は、
`openvins_imu_history_depth:=64 openvins_imu_input_diagnostics:=true`を追加します。
Patasmonkey共通launchの履歴深さの既定は32件、診断の既定はOFFです。
OpenVINS単独launchの既定値は別なので、共通launch以外では明示してください。
BEST_EFFORTと画像QoSは変更せず、
native consoleの`[IMU_INPUT_GAP]`とbagのIMU header stampを照合します。
64件は125 Hzで約512 ms分の比較用設定で、運用推奨値として確定していません。

```bash
python3 ~/ros2_ws/src/hwt905_rs485_driver/tools/run_imu_startup_trials.py --mode bringup --output /tmp/wit_startup_bringup
```

出力先は未作成のディレクトリを指定します。各bagの`wit_timing.csv`と
`wit_timing.json`に起動節目・取得時間を残し、出力先の`comparison.json`へ
0〜5秒、5〜10秒、10〜20秒、20秒以降の周波数・長い間隔・エラーをまとめます。
時刻基準はPython node初期化開始です。センサ内部の測定時刻ではありません。

## console診断の負荷を通常走行から切り離す

通常のcore/mission起動は`openvins_verbosity:=WARNING`、
`vio_adapter_diagnostics:=false`とする。従来はOpenVINSのDEBUGと、VIO変換の
約20行の診断を0.1秒ごとに常時出していた。consoleのpipe転送・launch側の整形と
ファイル保存の負荷を減らす変更であり、VIOの積分、画像処理、QoS、bag対象topic、
native状態/timing保存は変えない。異常警告・エラーは残す。

詳細な初期化判定や座標変換を調べるときは、従来相当へ明示的に戻せる。

```bash
ros2 launch pm_bringup pm_core.launch.py \
  openvins_verbosity:=DEBUG vio_adapter_diagnostics:=true \
  vio_adapter_diagnostics_interval:=0.1
```

単なるログ削減を、欠測やVIO停止の根治と解釈してはいけない。
比較試験ではGUI・bag・計測条件を揃え、停止済みbagのheaderと診断情報を照合する。

## IMU購読履歴の有限な受信余裕

VIO入力が181.6 ms空いた停止再現では、その区間のIMUがbagには保存されていた。
約126 Hzで約23件分に相当するため、共通launchは32件（約254 ms）の履歴を採用する。
5件は約40 msしかなく、今回観測した受信停滞をカバーしない。32件の比較試験2回で
VIO継続を確認したが、過負荷の根治・任意の長時間停止への保証ではない。
27件の追加CDR payloadは概ね9 KiB（DDS内部管理領域は別）で、画像queue・
記録cache・RAM bagは増やさない。実センサ欠測や消えた積分端点を補間でごまかさず、
既存の安全検査と非復帰ラッチを維持する。
従来条件へは`openvins_imu_history_depth:=5`で戻せる。
# 追加診断は既定OFF

通常起動はUDPv4＋SHM 2 MiB/participant。OpenVINSのnative状態・処理時間・
追加consoleファイル・parameter dumpは、それぞれ
`openvins_save_total_state`、`openvins_record_timing`、`openvins_console_log`、
`openvins_parameter_snapshot`を`true`にしたときだけ保存する。
以下に記すnativeファイル一覧は診断を有効にした場合のもので、通常は作成されない。
通常ROSトピックとOAKのセンサmetadata、運用警告・再現用設定保存は維持する。
詳細は`README_recording.md`の「通常運用のDDS・診断既定」を参照。
# OAK時計変換の切替と監視

共通launchの `oak_timestamp_mode` は既定 `host_clock`（現在時計の対応付け）です。
`legacy_first_message` は従来の最初のデータ基準へ戻すための選択肢です。
試験方式は現在のDepthAI時計とROS時計を対応付け、全OAK画像・IMUで共有します。
depth/confidence/disparityのHeader対応とRGB/CameraInfoの対応は変更しません。
生のdevice timestampやaccel/gyro個別timestampは診断側にそのまま残します。

```bash
# モータ・joystick・teleopなしのセンサ試験
ros2 launch pm_bringup pm_core.launch.py oak_timestamp_mode:=host_clock \
  use_vehicle_interface:=false use_joy:=false use_teleop:=false
```

ロールバックは同じ引数を `oak_timestamp_mode:=legacy_first_message` にして再起動します。
設定の動的切替は行いません。実機カメラのsimulated timeや実行中のROS時計ジャンプは
この試験方式の検証対象外です。実効方式・オフセットはdevice_infoとbag provenanceに残ります。

時計のずれ監視は既定ON、既存device_info診断で5秒ごとに記録します。Headerは自動補正しません。
`oak_clock_monitor_enabled:=false` で監視だけOFFにできます。
`oak_clock_warn_threshold_ms:=5.0` は時計差変化の警告閾値、
`oak_clock_max_reference_span_ms:=1.0` は時計読み取り幅の信頼上限です。
どちらも正の有限値で、VO許容誤差として実証した値ではありません。
診断のWARNは時計差超過と測定不確実を区別し、復帰後も累積回数・最大差を残します。
診断topicは既存bag対象なのでtopic一覧の追加は不要です。

## OpenVINS保護状態の軽量記録

mission bagは `/ov_msckf/diagnostics/health` を常時記録します。
OpenVINS側の対応版が必要です。DiagnosticArrayの状態は`initializing`／`running`／
`reset_required`で、1 Hz heartbeatと状態変化時の通知により、記録を途中から開始しても
保護状態を確認できます。必要だったIMU区間と判定時buffer端点も保持します。
過去bagの状態を遡って補完する機能ではありません。自動リセットも行いません。
診断が途絶えた場合と、ERROR状態が届いた場合は区別してください。
running表示だけではVO更新の継続や精度を保証しないため、VO出力の鮮度も確認します。
高レートの画像・IMU購読probeを増やさず、診断は数百bytes〜約1 KB/件の低容量です。
