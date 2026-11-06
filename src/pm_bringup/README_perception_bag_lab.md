# bagによる現行環境認識・ローカルパス確認

オンライン版とは別の`perception_bag_lab.launch.py`を使う。
bag player、local EKF再計算、現行perception、表示用MPPI、設定GUI、RVizをまとめて起動する。
実カメラ・Witドライバ・ODrive・Joy・teleop・走行管理は起動しない。
bag内の速度指令、旧`/tf`、旧`/odometry/local`、旧hazardも再生しない。
既存の`bag_clock_player`やオンライン版の起動方法は維持している。

## 起動コマンド

Foxy環境でROS→外部workspaceをsourceした後、追加packageをbuildする。
開発PCでは`patasmonkey_foxy_dev`コンテナを使い、GUIを表示できる環境で実行する。

```bash
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
cd /workspaces/patasmonkey_ws
colcon build --symlink-install --packages-select pm_evaluation pm_perception pm_planner pm_gui pm_bringup
source install/setup.bash
```

これまで評価していた7月bagは、CameraInfoのない旧OAK bagなので次の一発で起動する。

```bash
ros2 launch pm_bringup perception_bag_lab.launch.py \
  bag:=/workspaces/patasmonkey_ws/bags/rosbag2_2026_07_26-09_17_38 \
  old_bag:=true
```

CameraInfoを含む通常のbagは以下でよい。MCAP・SQLiteのbagディレクトリを指定する。

```bash
ros2 launch pm_bringup perception_bag_lab.launch.py bag:=/absolute/path/to/bag
```

既存のROS stackや別bag playerは止めてから起動する。
同一ROS domainでオンライン系と同時起動すると、同じtopic・TF・`/clock`が衝突する。

## 入力と再計算

必須topicは`/wheel/odometry`、`/wit/imu`、`/oak/depth/image_raw`とVIO。
`/ov_msckf/odomimu`があれば現在のrigid VIO adapterで`/vio/odometry`へ変換する。
なければ記録済み`/vio/odometry`を使い、どちらの場合も現在のgate・水平EKF・
姿勢高さobserver・composerでlocal odometryと`odom→base_link`を再生成する。
OpenVINS本体をステレオ画像から再実行するツールではない。
wheel odometryも記録済みを入力とし、motor_stateからの再計算ではない。
古いMotorStateはmessage定義が現在と異なる場合があるため、無条件にdeserializeしない。

|引数|既定値|用途|
|---|---|---|
|`vio_source`|`auto`|raw odomimuを優先。`raw`／`recorded`で明示選択|
|`old_bag`|`false`|照合済み旧OAKのintrinsics fallbackとoptical frame補正|
|`params_file`|pm_configのplanner_perception.yaml|現在の通常運用設定|
|`rate`|`1.0`|0.1〜4倍。GUIからも変更可能|
|`start_paused`|`false`|trueならGUIの再生操作まで停止|
|`duration_sec`|`0.0`|bag秒での再生長。0は終端まで|
|`reference_mode`|`current_pose`|その地点・向きから仮想的に直進。`fixed_start`で初期直線固定|
|`gui`／`rviz`|`true`|表示の個別無効化|

CameraInfoがなければ、既定では起動前にエラーを出す。
`old_bag:=true`は以前EEPROMと照合した7月OAK用の値を既存設定から使う明示指定であり、
任意のカメラに正しい校正を推定する機能ではない。
旧設定のfx/fy/cx/cy・640×400解像度と、`rgb_camera_optical_frame`を使用する。
別個体・別解像度のbagへ安易に指定しない。
固定TFはbagを優先し、bagになければURDFを使う。
raw VIO変換の取付TF・camera TFもbag／URDFの記録・設定に依存する。

## 表示と操作

オンライン用と共通のRViz設定で、選択costmap、青い直進参照、速度で色分けしたローカル予測、
原寸ロボット、カラー画像を表示する。
線幅はRVizの対象PathでLine Style=Billboards、Line Width（m）から変更できる。
既定は青い参照線8cm、速度色付きローカルパス12cm。
offlineの既定は**現在の再生地点・車体向きから毎回直進参照を作る**。
記録経路が曲がった後も、その場から直進を仮定した回避候補を見られる。
実績経路そのものやGNSS経路を追従する予測ではない。
GUIの「直進参照の作り方」で初期直線固定へ切り替えることもできる。

GUIで次を操作できる。

- bagの再生・一時停止、速度変更、経過時間表示。
- obstacleのみ／terrain込みのcost選択。
- 認識閾値・depth範囲・stride・map寸法・解除条件の変更。
- 予測の目標速度・車体半径・footprint追加余裕・直進参照の長さの変更。

footprintの初期値も`pm_config/config/planner.yaml`から取得する。
現在は半径0.3m＋余裕0.1m＝膨張半径0.4mで、解像度とは独立。セル幅の再加算はしない。
GUIはnodeから両方の実効値を読み込み、編集欄と合計を表示する。

認識パラメータの適用はmapを消去する。停止中に変更した場合、新しい観測結果は再生再開後に得られる。
cost選択だけの適用はmapを消去しない。変更前と同じ履歴で厳密に比較したい場合は、
設定を別YAMLへ保存し、launchを終了して最初から再生し直す。

全nodeはbagの`/clock`を使う。player自身だけはwall timerで進む。
低速再生・pauseに対するpreviewの入力timeoutもsim時刻で計算する。
巻戻し／loopはEKFやmapの状態を壊さないよう未対応。再試行はlaunch全体を再起動する。

終端ではplayerを維持し、最終clock・mapを残して確認できる。
ただし最後のclock以降に必要なTFがない画像まで、強制的に融合完了させるわけではない。
GUIを閉じるかCtrl-Cで全体を終了する。headless時も終端で自動終了はしない。

## ローカルパスが出ない場合

速度・内部予測窓の初期値は`pm_config/config/planner.yaml`から読む。
現在は目標1m/s、25 steps × 0.1秒 = 2.5秒。可視化更新2Hzとは別の値である。
GUIは実効値を取得し、予測窓と、成功時には実際の軌道長を表示する。
目標速度×時間の2.5mが、そのまま予測軌道長になるわけではない。
実測速度・加減速制約・最適化の収束等で短くなる。

unknown・危険・map外を通る候補は採用しない。特にカメラで未観測の車体直下や
footprint周囲のunknownで、予測線が出ないことがある。
前進モデルの範囲外の速度・後退・大きな旋回でも、その理由を表示して予測を消す。
有効でない古い予測線を残したり、unknownをfreeとして補完したりはしない。

この表示は「その地点から走行可能な経路を必ず見つける」という保証ではない。
現在の認識・モデル・footprint条件で予測が成立するかを確認するツールである。
地図の保持・解除規則は[オンライン版README](README_perception_lab.md)と同じ。

## 実装と診断topic

- `pm_evaluation/perception_bag_player.py`: 制御topicを除外したstream再生、pause・rate、clock・static TF保持。
- `pm_bringup/launch/perception_bag_lab.launch.py`: 入力検査とsensorless再計算graph。
- `pm_gui/perception_lab.py`: offline操作を追加した共通パネル。
- `pm_planner/perception_preview.py`: sim時刻timeout、直進参照mode。

`/perception_bag_player/status`にstate・rate・経過秒・送出数・最大スケジュール遅延をJSONで出す。
これはplayer送出件数であり、mapperの採用／融合完了数の保証ではない。
画像の全frameや全mapをメモリへ蓄積しない。SQLiteは読み取り専用で開き、元bagを変更しない。

計画入力を診断保存する場合だけ`planning_probe_dir:=/absolute/path/new_directory`を指定する。
実際に使ったcost・pose・速度・参照・乱数・最適化直前の状態を保存する。
通常の表示には不要で、ディスクI/Oを追加するため性能測定には混ぜない。

## オンライン・オフライン共通の速度表示

両launchは同じ`pm_planner/perception_preview`と`pm_gui/perception_lab`を使用する。
以下の表示・変更は両方に反映される。起動コマンドの追加引数は不要。

- RViz：採用軌道だけを赤（0 m/s）→黄→緑（車体設定の最大速度）の固定範囲で色分け。
  白文字で約0.5秒間隔と先頭・終端の予測時刻／速度を表示する。
- GUI：横軸は現在位置から各制御区間の開始位置までの軌道距離[m]、縦軸は速度[m/s]。
  水色実線=採用速度、橙破線=障害物／unknownとの距離による速度上限、
  灰点線=設定目標速度、桃一点鎖線=現在のodometry速度。
  先頭・終端の採用速度と実測速度は数値でも表示する。
- 車体半径、footprint追加余裕、目標速度に加え、`clearance_distance_m`、
  `clearance_cost_weight`、`clearance_min_speed_mps`をGUIで変更可能。
  初期値は`pm_config/config/planner.yaml`から読み、nodeの実効値をGUIへ取得する。

橙の上限は各区間開始位置でsolverと同じ式から求める。加減速制約のため
採用速度が上限を一時的に上回る場合があり、表示では両者を隠さず区別する。
`clearance_min_speed_mps`は余裕不足時の上限設定であり、最低走行速度ではない。
停止・終点・他の制約により、実際の採用速度はこれより低くなる。
速度色はhazardの白／黒とは別の意味で、赤線が障害物を示すわけではない。

`/perception_preview/speed_markers`は`visualization_msgs/MarkerArray`、
`/perception_preview/speed_profile`はGUI向け`std_msgs/String`のJSON。
JSONの`distance_m`、`speed_mps`、`cap_mps`は同じ制御区間順。
無効な入力／予測失敗では線・ラベルとグラフを消す。
従来の`local_path`は継続配信し、RVizでは重なりを防ぐため単色Pathを既定OFFとする。
線幅はMarkerの`scale.x=0.12 m`で設定し、従来PathのLine Widthは色付き線に作用しない。
表示用の採用軌道だけを約2Hzで生成し、MPPI候補全体は配信しない。
この可視化は速度指令を出さず、Joy操縦を制限しない。
GUIのYAML保存は認識設定のみであり、plannerの変更を永続化する場合は正本を編集する。
