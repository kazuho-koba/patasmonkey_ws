# 制御・必須記録・任意記録を分離した起動

## 遠隔カメラ表示

OAK起動時は `oak_preview_node` も別プロセスで起動し、GUI向けに
`/oak/preview/image/compressed` を最大5 Hz・幅160 px・JPEG quality50で配信します。
`use_oak_preview:=false` で無効にできます。原画像／IMUのtopic、QoS、missionの
保存範囲は変更していません。表示用JPEGは既定でbag保存しません。
新しいpm_gui設定とdepthai_driverを両workspaceでbuild／sourceしてください。

## 記録の分離

OAK confidence・disparity・実効校正／設定JSONはmission側に追加しました。
topic座標の違い、採用ground画素の分布解析、厳格閾値の再処理手順は
[confidence記録・解析README](../pm_evaluation/README_depth_confidence.md)を参照してください。

`pm_core.launch.py`はセンサ、motor/vehicle interface、teleop、localization、
perceptionを起動します。`mission_bag.launch.py`は上位の観測・オンライン状態・
指令/安全履歴、`debug_bag.launch.py`は再計算可能な派生表示だけを記録します。
coreとrecorderの停止は独立です。**bagを止めても車両制御は止まりません。**

旧`pm_bag_global_localization.launch.py`は従来の一括記録・既存引数を維持します。
旧launchと新coreを同時に起動しないでください。実装共通部分は`stack_launch.py`です。

## 起動例

### 単一bagの非同期・圧縮比較（検証用）

`writer_backend:=async`は標準Foxy transportを維持し、保存を有限FIFOと
専用threadへ分離する。既定は`standard`であり、自動では切り替えない。
`writer_queue_bytes`はCDR payload合計の上限[byte]（既定64 MiB）で、
`writer_queue_items`はmessageとtopic登録/削除を含むitem上限（既定8192）です。
byte/item双方を制限します。byteだけ増やしても小さいmessageが多数滞留すると
item上限に達するため、停止後の統計でどちらの上限が効いたか確認してください。
容量超過は録画失敗であり、途中bagを成功扱いしない。
受理済みmessageの保存完了だけでは試験全期間の記録成功にはならない。

`mcap_preset_profile:=zstd_fast_crc`は、対応する外部workspaceのMCAP overlayで
可逆圧縮とCRCを有効にする。未対応overlayは起動前に拒否する。
既定の`default`は従来の非圧縮設定で、実験用環境変数の継承も解除する。
bulk分割との併用を避け、単一bag比較では`bulk_sidecar:=false`を明示する。
異常なディスク待ちは大きなFIFOや圧縮だけでは解消しない場合がある。
詳細と限界は[pm_recording README](../pm_recording/README.md)を参照する。

```bash
# 記録のみ。センサcoreは安全な設定で別途起動する。
ros2 launch pm_bringup mission_bag.launch.py writer_backend:=async \
  writer_queue_bytes:=134217728 mcap_preset_profile:=zstd_fast_crc bulk_sidecar:=false
# 元の単一bag方式へ戻す：writer_backend:=standard mcap_preset_profile:=default
# 以前の分割方式へ戻す：上記にbulk_sidecar:=trueを追加
```

各terminalでFoxy→外部workspace→主workspaceの順にsourceします。開発コンテナでは
workspace引数を`/workspaces/patasmonkey_ws`と`/workspaces/ros2_ws`へ変更します。
この標準mountが存在する場合は既定値も自動でそのパスになります。

```bash
# Terminal 1: 先に必須記録を開始。Jetsonホストの既定workspaceを使用。
ros2 launch pm_bringup mission_bag.launch.py

# Terminal 2: 安全な静止確認用。実走行時だけ安全確認の上でmotor/teleopを有効にする。
ros2 launch pm_bringup pm_core.launch.py use_vehicle_interface:=false use_teleop:=false

# Terminal 3: 必要な実験だけ追加。MISSION_BAGはTerminal 1に出た実bag path。
ros2 launch pm_bringup debug_bag.launch.py mission_bag:="$MISSION_BAG"
```

通常のcore既定は従来同様motor/teleopを含みます。安全が未確認なら上のOFF指定を使います。
coreのlocalization/mapper/センサ引数は旧一括launchと同じです。
coreの`record_bag`引数は互換表示だけで、trueでもrecorderは起動しません。

bag名は`rosbag2_YYYY_MM_DD-HH_MM_SS`が既定。missionは`~/patasmonkey_ws/bags/`、
debugはその`debug/`配下へ別bagとして保存します。`bag_directory`/`bag_name`で変更可能。
既存ディレクトリは上書きしません。debugとmissionは別bagで、debugだけでは入力の
再計算はできません。共通topicは二重に記録しないようprofileを分けています。

missionが起動前に存在しないcoreのparameterを直ちに取得できなくても、10秒間隔で
新しいnodeを探して取得します。各巡回最大10秒、service待ち1秒。停止時にも再取得します。
サービス非公開・タイムアウト・同名nodeの重複等は失敗情報として保存します。

## missionの記録履歴

### 保存完了は実ファイル件数も検査する

metadata.yamlとros2 bag infoだけでは、cacheの未保存等を検出できません。
停止後、MCAPは全message recordをdecodeせず走査し（存在するCRCも検査）、
SQLiteはread-onlyで構造・件数を確認して、metadataの総数・topic別件数と照合します。
不一致・ファイル欠如・読取不能ならcompletion.jsonのverifiedをfalseにし、
GUI/systemdへ成功を返しません。検査詳細はcontent_integrityに保存します。
追加IMU bagも同様に検査します。走査は全bagを読むため、大きいbagほど停止確認が長くなります。
この件数一致はpublish総数との一致を保証せず、DDS配送中の欠落検証とは別です。

Pythonのmcapライブラリが必要です（setup.pyに依存を記載）。storage pluginとは別の依存です。
検査依存が利用できない場合は保存成功と判定しません。

既存bagも元データを変更せず再検査できます。不一致時の終了コードは1です。

```bash
ros2 run pm_bringup verify_bag_integrity --bag /path/to/bag --output /path/to/new_check.json
```

### Foxy書き込みcacheの制約

このFoxyのmax-cache-sizeはbyteではなくmessage件数です。Jetsonの実ヘッダーで確認しました。
非zero値を使ったMCAP試験ではmetadataに82,679件があるのに実messageが0件となりました。
当該環境ではMCAPの非zero cacheを起動前に拒否し、0（直接書き込み）を維持します。
max_cache_messages launch引数はこの単位です。MCAPでは0以外を指定しないでください。
cacheを有効にすれば非同期writerになると仮定してはいけません。

```bash
ros2 launch pm_bringup mission_bag.launch.py max_cache_messages:=0
```

missionの既定QoSはbag_qos_mission_buffered.yamlです。実測の約0.5～0.7秒の
記録停滞を根拠に、左右画像20件、RGB/depth等10件（約1秒分）、
IMU200件、local EKF100件等の有界履歴を使用します。publisherやOpenVINSのQoSは変更しません。
明示対象以外は自動QoS、debugは従来のbag_qos.yamlのままです。
RAM・記録遅延との交換条件であり、完全無欠測や常時低遅延は保証しません。
旧一括launchのrecorderにはこの既定変更は適用されません。

旧設定へ戻す場合はqos_profile_overrides_pathにpackage内のbag_qos.yamlの実パスを指定します。
実際に使用した設定は各bagのprovenance/bag_qos.yamlへコピーされます。

## IMU専用bagの追加記録（移行・検証用）

画像・点群を扱うrecorderの処理停滞からIMU記録を分けたい場合は、次を指定します。
既定はOFFで、通常missionのtopic構成を変えません。

```bash
ros2 launch pm_bringup mission_bag.launch.py imu_sidecar:=true
```

通常bagと同じ親ディレクトリに、名前末尾が\`_imu\`の専用bagと
\`_imu_record.log\`を保存します。専用bagには\`/oak/imu/data\`と\`/wit/imu\`だけを
RELIABLE / VOLATILE / KEEP_LAST 200、cacheなしで記録します。
履歴は約1.6秒（OAK 125 Hz）／2秒（Wit 100 Hz）の待機容量であり、欠測を保証する値ではありません。
両topicは通常bagにも残すため、追加負荷・重複保存があります。

停止要求は両recorderへSIGINTを送り、両方の終了、metadata、bag infoを確認します。
mainのcompletion.jsonに専用bagの検証結果も含め、一方でも失敗したらGUI/systemdへ成功を返しません。
対応先・実コマンド・実QoSはmainのprovenance/imu_sidecar.jsonとimu_sidecar_qos.yamlに保存します。
両ディレクトリを一組として保管してください。絶対パスは移動後に読み替える必要があります。
起動直後の取消で専用bag保存を確認できない場合も、安全側にERRORとします。

再生時に両bagのIMUを同時publishしないでください。解析はpm_evaluationの
tools/compare_live_imu_bags.pyで共通header区間を比較できます。
GUIの既存起動設定と旧一括launchには自動適用しません。mission launch引数で明示します。

## 同梱する再現情報

```text
rosbag2_.../
  *.mcap, metadata.yaml
  bag_info.txt, completion.json
  provenance/
    recording.json               # topic/profile/環境/対応mission bag
    bag_profiles.yaml, bag_qos.yaml
    effective_parameters.json    # nodeごとの実効値・取得時刻・未取得/失敗
    core_manifest.json           # pm_coreの実効launch引数・付帯ログpath
    versions.json, git/repo_*/    # HEAD/branch/直近10commit/status/差分/小さな未追跡src
    installed_packages.json, installed_sources/ # 実overlay・実行ファイルhash・Python内容
    configuration/, configuration_index.json
    runtime_artifacts/            # core付帯ログの停止時点snapshot
```

Git履歴はsource treeの情報です。未commitのtracked差分はHEADとの差でstaged/unstagedを
含み、ソース/設定は1ファイル1 MiB以下を保存します。bag/build/local notesやprivate設定は
除外し、未追跡ファイルの未コピーもmanifestに明記します。
ソースHEADだけで実際のビルド済みバイナリを証明できるわけではありません。
主workspaceと外部workspace、および外部src直下のGit repoを対象にします。
使用したinstall側設定はhashと内容を保存し、OpenVINS相対校正も追跡します。

認証parameterは一般的なpassword/token等の名前で伏せ、private/credentialファイルを
コピーしません。名前で判別できない秘密や差分内の埋込値は自動で完全検出できないため、
bagを共有する前にはprovenanceを確認してください。除外により完全再現不能な項目も残ります。

`/parameter_events`は途中変更の記録で、初期parameter snapshotの代わりにはなりません。
snapshotはRPC取得時の値で、全nodeの同時刻atomicallyなsnapshotではありません。
core_manifest未取得ならその旨のJSONを残し、起動条件の完全保存に成功したと扱いません。
core付帯ログは`~/.ros/pm_core_sessions/<bag_name>/`へ分離して保存します。
recorder停止時点までコピーするため、coreがその後も稼働したログ末尾は含みません。

### 記録経路の比較診断

`polling_interval_ms`は未発行topicの探索周期です（既定100 ms）。
`mission_bag.launch.py polling_interval_ms:=1000`で探索負荷だけを比較できます。
既存topicの取得周期は変えませんが、起動が遅いtopicの購読開始は最大約1秒遅れます。
`qos_profile_overrides_path:=/絶対パス/比較設定.yaml`ではbag側の購読QoSだけを
比較できます。空なら従来の`config/bag_qos.yaml`を使用します。
`max_bag_size_bytes`は分割サイズ[byte]です（既定2000000000、10進の2 GB）。0で分割を無効化し、
分割/close処理による停滞を比較できます。無分割では長時間試行の1ファイルが大きく
なるため、通常運用へ変更する前に保存容量・破損時の復旧・取り扱いを検討してください。
実行commandと実際に使用したQoS YAMLはprovenanceへ保存します。
RELIABLEへの変更はpublisherの待ちを増やす可能性があるため、むやみに変更しないでください。

比較用の`config/bag_qos_imu_buffered.yaml`も用意しています。bag側のIMU二topicだけを
RELIABLE・KEEP_LAST 200へ設定します（publisherとOpenVINSは変更しません）。
125 HzのOAKでは約1.6秒、100 HzのWitでは約2秒の履歴であり、実測の秒単位停滞に
対して保持余裕を増やす試験値です。再試験では多くのIMU欠測が減りましたが、
約1.5～2秒の滞留で履歴が埋まり欠測も残ったため、通常の既定にはしていません。

独立したIMU到達プローブは`pm_evaluation/tools/probe_imu_delivery.py`です。
Foxy→外部→主workspaceをsourceし、次のように別terminalで実行します。

```bash
python3 src/pm_evaluation/tools/probe_imu_delivery.py --output /tmp/imu_delivery.csv
```

`/oak/imu/data`と`/wit/imu`のheader/受信時刻をメモリ保持し、Ctrl-CでCSVへ保存します。
BEST_EFFORT・履歴200件、最大60000件で、画像を購読せずモータ指令も送りません。
保持上限超過は`discarded`へ計上します。終了時に0件であることを確認してください。
追加購読自体にもCPU負荷があるため、A/B条件で同じように起動してください。
プローブとbagの両方のheader欠測を、ドライバ側の取得/publish診断とも照合します。
`--odometry-topic /ov_msckf/odomimu`でVIO出力のheaderも追加でき、保護発動後に
新しい推定が公開されていないかを再生試験で確認できます。

実機センサを起動せずrecorderだけを比較する場合は、
[pm_evaluationの再記録照合手順](../pm_evaluation/README_recording_delivery.md)を使用できます。
元bagの選択messageと再記録payloadを照合するため、既存の欠測を新規欠測と混同しません。
専用ROS domainでIMUのみ／画像あり、MCAP／SQLiteを比較できますが、全missionや
実機publisherのQoSをそのまま再現する試験ではない点に注意してください。

`pm_evaluation/tools/probe_process_wait.py --pid <recorderの実PID> --output /tmp/recorder_wait.json`
では、main threadの`wchan`、`schedstat`、processのI/O累積値、NVMe/dirty pageを
既定10 Hzで読み取れます。PID再利用を検査し、対象processへsignalを送りません。
kernelの権限設定で読めない項目はerrorとして残します。system全体のdisk値だけで
recorder自身のI/O待ちと断定せず、bag欠測時刻とprocessの待ち状態を照合してください。

Foxy 0.3.11の`--max-cache-size`は**メッセージ件数**で、現行ROS 2のbyte単位の
設定例をそのまま使ってはいけません。上流実装ではbatch書き込みも同期実行で、
reset時の端数cache書き出しも確認できないため、本構成では従来どおり0を維持しています。
cache増量を非同期化や欠測対策とみなさず、利用前に実装と終了時の件数を検証してください。
根拠は[上流0.3.11のSequentialWriter実装](https://github.com/ros2/rosbag2/blob/0.3.11/rosbag2_cpp/src/rosbag2_cpp/writers/sequential_writer.cpp)です。

SIGINT時は今回のrecorder childだけを停止して待ち、metadataとbag infoを確認します。
`completion.json`の`verified`を確認してください。電源断/SIGKILLでは完了保証はできません。
metadata snapshot処理用にlaunchの強制停止待ちを長くしています。

## topic分類

全topicの正本は[config/bag_profiles.yaml](config/bag_profiles.yaml)です。
上位/下位分類に従い、下位でも実走行で保存推奨の`/wheel/odometry`だけmissionへ昇格。
`/imu`は重複か未確定なのでmissionに残します。未発行topicを記録するためノードを
追加起動することはありません。将来plannerがhazard/costを実入力に使う場合は、その
判断入力をmissionへ昇格する必要があります。

`/tf_static`と`/robot_description`はtransient_local QoSで初期値の取り逃しを抑えます。
CameraInfoは今回毎フレーム保存を維持し、固定値の間引きは実装していません。
任意bagを止めてもdebugの生成/配信自体が必ず止まるわけではありません。

### 必須：mission

| 分類 | topic（それぞれ個別に記録） |
|---|---|
| カメラ観測 | `/oak/stereo/left/image_raw`, `/oak/stereo/right/image_raw`, `/oak/color/image_raw`, `/oak/depth/image_raw`, `/oak/imu/data` |
| 外部IMU | `/wit/imu`, `/wit/mag`, `/wit/mag_corrected`, `/imu` |

| 車両実測・車輪入力 | `/motor_state`, `/wheel/odometry` |
| GNSS | `/fix`, `/fix_velocity`, `/navpvt`, `/navrelposned`, `/navheading`, `/navstatus`, `/navstate`, `/navclock`, `/navsvin`, `/monhw` |
| RTK | `/rtcm`, `/rxmrtcm` |
| VOオンライン状態 | `/ov_msckf/odomimu`, `/ov_msckf/points_msckf`, `/ov_msckf/points_slam` |
| 自己位置推定 | `/odometry/local`, `/odometry/local_horizontal`, `/odometry/local_vertical`, `/odometry/global` |
| 操作・安全・外部初期化 | `/pm/joy`, `/cmd_vel_joy`, `/cmd_vel`, `/sim_cmd_vel`, `/emergency_stop`, `/set_pose` |
| OAK metadata | `/oak/diagnostics/left_frame`, `/oak/diagnostics/right_frame`, `/oak/diagnostics/color_frame`, `/oak/diagnostics/depth_frame`, `/oak/diagnostics/imu_packet`, `/oak/diagnostics/device_info` |
| 実行時診断 | `/diagnostics`, `/rosout`, `/vio/vertical_gate/diagnostics`, `/vio/twist_gate/diagnostics`, `/gnss/fix_gate/diagnostics` |
| 初期化 | `/localization/initialization/status`, `/odometry/gps` |
| parameter履歴 | `/parameter_events` |
| 座標・geometry | `/tf`, `/tf_static`, `/oak/color/camera_info`, `/oak/depth/camera_info`, `/robot_description` |

`/wit/mag_corrected`は`/wit/mag`のheader・frameを保持し、pm_configの
`hwt905_mag_calibration.yaml`に記載した暫定biasと軸別scaleを別プロセスで
適用した磁気ベクトルです。生の`/wit/mag`も保存します。既存ドライバが使う
raw register単位であり、MagneticField型の標準単位Teslaには換算されません。
方位offset・偏角や傾斜補償はベクトルに適用せず、GUIの方位計算時に適用します。
水平面の静止データから求めた暫定値はz bias未同定で、回転・傾斜・実走行では
未検証です。bag終了後の設定記録は終了時点の設定で、実行中に書き換えた場合
ノード起動時の値と異なり得るため、記録中に校正ファイルを変更しないでください。

### 任意：debug（22 topic）

| 分類 | topic（それぞれ個別に記録） |
|---|---|
| elevation表示 | `/depth_elevation_mapper/elevation_debug`, `/depth_elevation_mapper/relative_elevation_debug`, `/depth_elevation_mapper/elevation_variance_debug`, `/depth_elevation_mapper/observation_count_debug`, `/depth_elevation_mapper/observation_age_debug` |
| terrain表示 | `/depth_elevation_mapper/obstacle_height_debug`, `/depth_elevation_mapper/slope_debug`, `/depth_elevation_mapper/roughness_debug`, `/depth_elevation_mapper/step_height_debug`, `/depth_elevation_mapper/terrain_hazard_debug`, `/depth_elevation_mapper/terrain_hazard_cause_debug` |
| VO描画・軌跡 | `/ov_msckf/trackhist`, `/ov_msckf/poseimu`, `/ov_msckf/pathimu` |
| VIO派生 | `/vio/odometry`, `/vio/odometry/gated`, `/vio/odometry/twist_gated` |
| GNSS/IMU gate派生 | `/fix/gated`, `/wit/imu/heading_calibrated` |
| GNSS変換派生 | `/odometry/gps`, `/gps/filtered` |
| RobotModel表示 | `/joint_states` |
# 画像・点群の排他的分離（任意の比較モード）

`mission_bag.launch.py bulk_sidecar:=true`、または`record_trial --bulk-sidecar`で、
左右・RGB・depth・confidence・disparity画像とOpenVINSの2種の点群を
`<main-bag>_bulk`へ移します。IMU・EKF・GNSS・TF・診断・操作履歴はmainに残します。
topicの省略や重複ではなく、全mission集合を2つのwriterに割り当てます。
未知の新topicはmainに残し、暗黙の未記録を防ぎます。
分離側はbag_qos_bulk_buffered.yamlを使い、実測約0.9秒の停滞に対して
約1.5秒の履歴（mono/点群30件、RGB-D等15件）を保持します。mainのQoSは変更しません。
1秒から50%の増量で、数百件への一律増量ではありません。遅延とRAMは増え得ます。

目的は、大きい画像のserialization/writeと軽量topicの記録処理を別executorにする比較です。
CPU・ディスク・DDSは共有するので資源隔離や無欠測保証ではありません。
既定はfalse、debugでの使用とimu_sidecarとの同時指定は拒否します。
従来の単一bagの構成・GUI操作・topicのROSインターフェースは既定で変えません。

mainの`provenance/topic_partition.json`に両bagの場所と担当topicを保存します。
停止時は両所有childへSIGINTを送り、両bagのmetadata・bag info・実件数を検査し、
mainのcompletion.jsonは両方成功した場合だけverified=trueになります。
第二bagの結果はbulk_sidecarに記録し、従来のimu_sidecarフィールドと区別します。

**分離時はmainだけでは全データが揃いません。両bagを一緒に保管してください。**
既存の単一bag前提の解析・再生へ無条件に渡さないでください。
全topic監査はaudit_bag_delivery.pyの--additional-bagで排他的分離を合算できます。
重複保存型IMU sidecarの合算は拒否します。再生の統合は別途必要であり、
2つのros2 bag playを同時起動するだけでは時刻の同期を保証できません。
そのため本モードは現時点で任意の比較用で、通常記録の既定へは採用しません。
# 通常運用のDDS・診断既定（2026-10-13）

`pm_core`とmission/debug recorderは、UDPv4＋SHM **2 MiB/participant**を既定で使う。
missionの既定購読QoSはRELIABLE、有界履歴のまま。画像解像度・周波数は変えない。
この容量はbag保存用RAMではなく、同一ホスト内の通信bufferである。
launchで`dds_profile:=/absolute/path/to/profile.xml`を指定すると比較・復帰できる。
親shellやOS全体の環境、systemd unitは変更しない。GUI/CLIから同じlaunchを
起動すれば同じ既定が適用される。任意の`ros2 bag record`には自動適用しない。

OpenVINSのnative状態、timing、追加consoleファイル、実効parameter dumpは既定OFF。
必要時だけ以下を指定する（core側の引数）。

```bash
ros2 launch pm_bringup pm_core.launch.py \
  openvins_save_total_state:=true openvins_record_timing:=true \
  openvins_console_log:=true openvins_parameter_snapshot:=true
```

Wit/OAK timing、OpenVINS IMU gap、mapper callback/executor、VIO adapterの詳細診断も
既定OFF。DDS/rcl受信hookは`dds_receive_diagnostics:=true`を明示した場合だけ
既存の`LD_PRELOAD`/`PM_DDS_TRACE`/`PM_RCL_TRACE_PATH`を継承する。
既定OFFでは今回のhook libraryだけ除去し、他のpreload libraryは維持する。
通常のROS出力、センサmetadata、健全性ゲートの警告、bag保存完了・件数検査、
再現用provenanceは運用データとして維持する。sidecar、async writerは既定OFFのまま。

## メタデータ収集時期の比較

mission recorderの `metadata_capture_timing` は既定 `after_recording`（終了後収集）です。
`during_recording` を明示すると従来の記録中収集へ戻せます。

```bash
ros2 launch pm_bringup mission_bag.launch.py metadata_capture_timing:=after_recording
```

終了後方式では、writer終了を待ってからGit状態・インストール済みソース・ノードパラメータ・
参照設定ファイルを収集します。録画コマンド、QoS、DDS設定など軽量な録画条件は従来どおり
記録開始時に保存します。録画中は追加のmetadata node・パラメータ巡回を起動しません。
記録開始時の情報と取り違えないため、recording.jsonにpost_recording_only_not_start_snapshotという
scopeを明記し、取得日時付きのパラメータsnapshotとmetadata_deferred_phase.jsonを保存します。
終了時snapshotを開始時の状態と同一と保証する機能ではありません。

終了時のノード設定を残すには、**bagを停止した後もcoreを継続稼働させてください**。
coreを同時に停止すると、パラメータやカメラの校正snapshotを取得できない場合があります。
GUIの全停止では安全上vehicleを先に停止するため、車両nodeの終了時parameterは取得できない
場合があります。core由来の参照設定ファイルは保存できますが、動的な車両設定の開始時保証とは別です。
設定ファイルやソースが記録中に変更された場合、保存されるのは終了時のファイルです。
本番で厳密な開始時状態が必要なら、事前snapshotとの併用・変更履歴の検査が別途必要です。

終了処理は従来より長くなる場合があります。GUI/systemdのSTOPPING中に収集とbag検証を行い、
メタデータ収集の失敗はcompletion.jsonのmetadata_snapshot_errorへ保存します。
bag本体のclose・件数検査を省略しません。従来動作への復帰はduring_recordingを指定してください。

# Wit IMUの接続先とCPU優先度

## USB–RS485アダプタの切替

IMUの接続先は旧CH340/CH341系の
`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`を既定とします。共通launchの
`wit_port`で接続先のみを上書きでき、通信速度230400 baud、slave ID 80は保持します。
従来のCH341へ戻す場合は接続後に`ls -l /dev/serial/by-id/`で実際のパスを
確認し、例えば`wit_port:=/dev/ttyUSB0`を起動引数に加えます。
両方を同時接続しても自動選択はしません。番号は再接続で変化し得るため、
旧アダプタもby-id名を指定する方が確実です。
ドライバ単体起動ではYAMLの`port`またはROSの`-p port:=...`で切り替えます。
CP2102のby-id名はYAMLとlaunchにコメントで残しています。CP2102へ戻す場合は
`wit_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`
で切り替えられます。同じシリアル番号なしアダプタを複数接続するとby-id名が
衝突し得るため、この固定名は現在の1台接続を前提としています。

## CPU優先度の設定

共通起動構成（pm_coreおよびpm_bag_global_localization）はWit IMUだけを
`nice -n -5`で起動します。`wit_nice:=0`で従来の優先度へ戻せます。
他のノードや記録プロセス、ドライバ単体起動には適用しません。
niceは相対指定なので、親がnice=0の場合の実効値が-5です。
リアルタイムスケジューリングではなく、100 Hzの保証でもありません。

負のniceには権限が必要です。権限不足時は`nice`が警告して元の優先度で起動します。
Jetsonで`ulimit -e`が0なら、そのままでは-5になりません。
管理者がPAMログイン向けに`/etc/security/limits.d/90-pm-nice.conf`等へ
以下を設定し、再ログインしてから起動する方法があります。

```text
nvidia soft nice -5
nvidia hard nice -5
```

systemd起動はPAM設定だけでは変わらないため、対象unit側の`LimitNICE=25`が
別途必要です。既存の起動プロセスには遡及しません。
システム全体やPython実行ファイルへの権限付与は避けてください。
起動後は次の読み取り専用コマンドで実効値を確認します。

```bash
ps -C hwt905_imu_node -o pid,ni,args
```
