# 制御・必須記録・任意記録を分離した起動

`pm_core.launch.py`はセンサ、motor/vehicle interface、teleop、localization、
perceptionを起動します。`mission_bag.launch.py`は上位の観測・オンライン状態・
指令/安全履歴、`debug_bag.launch.py`は再計算可能な派生表示だけを記録します。
coreとrecorderの停止は独立です。**bagを止めても車両制御は止まりません。**

旧`pm_bag_global_localization.launch.py`は従来の一括記録・既存引数を維持します。
旧launchと新coreを同時に起動しないでください。実装共通部分は`stack_launch.py`です。

## 起動例

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

### 必須：mission（52 topic）

| 分類 | topic（それぞれ個別に記録） |
|---|---|
| カメラ観測 | `/oak/stereo/left/image_raw`, `/oak/stereo/right/image_raw`, `/oak/color/image_raw`, `/oak/depth/image_raw`, `/oak/imu/data` |
| 外部IMU | `/wit/imu`, `/wit/mag`, `/imu` |
| 車両実測・車輪入力 | `/motor_state`, `/wheel/odometry` |
| GNSS | `/fix`, `/fix_velocity`, `/navpvt`, `/navrelposned`, `/navheading`, `/navstatus`, `/navstate`, `/navclock`, `/navsvin`, `/monhw` |
| RTK | `/rtcm`, `/rxmrtcm` |
| VOオンライン状態 | `/ov_msckf/odomimu`, `/ov_msckf/points_msckf`, `/ov_msckf/points_slam` |
| 自己位置推定 | `/odometry/local`, `/odometry/local_horizontal`, `/odometry/local_vertical`, `/odometry/global` |
| 操作・安全・外部初期化 | `/pm/joy`, `/cmd_vel_joy`, `/cmd_vel`, `/sim_cmd_vel`, `/emergency_stop`, `/set_pose` |
| OAK metadata | `/oak/diagnostics/left_frame`, `/oak/diagnostics/right_frame`, `/oak/diagnostics/color_frame`, `/oak/diagnostics/depth_frame`, `/oak/diagnostics/imu_packet`, `/oak/diagnostics/device_info` |
| 実行時診断 | `/diagnostics`, `/rosout`, `/vio/vertical_gate/diagnostics`, `/vio/twist_gate/diagnostics`, `/gnss/fix_gate/diagnostics` |
| parameter履歴 | `/parameter_events` |
| 座標・geometry | `/tf`, `/tf_static`, `/oak/color/camera_info`, `/oak/depth/camera_info`, `/robot_description` |

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
