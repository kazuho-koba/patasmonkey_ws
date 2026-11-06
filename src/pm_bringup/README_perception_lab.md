# 現行環境認識とローカルパスの確認ツール

実機を起動せずbagから確認する場合は、別launchの
[オフライン版](README_perception_bag_lab.md)を使用する。

実センサ・wheel odometry・Wit IMU・local EKF・現行perceptionを使い、RVizと
設定GUIでcostmapとMPPIの予測を確認する。MPPIの実装は`pm_planner/MppiPoc`を再利用する。
実車の移動は手押しかJoyが担当し、preview側は速度・走行許可を一切publishしない。
通常の`execution_manager`は起動しない。Joyの手動指令に障害物回避を適用する機能ではない。

## 起動

対象UGVへデプロイ後、Foxy環境内で以下のpackageをbuildする。
開発PCでは`patasmonkey_foxy_dev`コンテナ内で実行する。

```bash
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
cd /workspaces/patasmonkey_ws
colcon build --symlink-install --packages-select pm_config pm_control pm_perception pm_planner pm_gui pm_bringup
source install/setup.bash
```

Jetsonホストではworkspaceパスをその配置に読み替え、ROS→外部workspace→main workspaceの順でsourceする。
ディスプレイのある環境で、次のコマンド一つでセンサ・認識・preview・GUI・RVizを起動できる。

```bash
# Joyによる遠隔操縦＋その時点の認識に基づくローカルパス表示
ros2 launch pm_bringup perception_lab.launch.py drive:=true
```

このモードではモーターが駆動可能になる。既存`pm_drive`の中立確認・非常停止・指令timeoutを使う。
経路は表示用であり、車体がその線を自動追従することはない。
通常Core・Drive・Operator Console等を別に立ち上げている場合は終了してから使用し、
カメラ、Joy、ODrive、TF、同名nodeの二重起動を避ける。

```bash
# 手押し用（既定）：Joyの入力取得は起動するが、teleop・通常vehicle interfaceは起動しない
ros2 launch pm_bringup perception_lab.launch.py
```

手押し用は専用`motor_state_observer`がODriveをIDLEにし、エンコーダを読む。
速度・gain・校正は書き込まない。エンコーダ計測のためODriveへの給電は必要。
手押しでタイヤが回転すればwheel odometryへ反映される。車体を持ち上げた移動では
車輪に変位が出ないため、同等の移動追跡は保証しない。

開始時は数秒静止させ、localの方位初期化とOpenVINS初期化を待つ。
このツールのOpenVINSは`try_zupt=true`で停止初期化を試みる。
暗所・特徴不足・センサ未接続などでは初期化できないことがある。
Witとwheelでlocal EKFは動くが、VIOの未初期化や高さgateの停止が認識精度へ影響し得る。
カメラのS/Nに合う校正を使うこと。別個体のカメラでは`vio_config_file`を適合する設定に変更する。

## 表示と操作

RVizは`odom`固定、`base_link`追従の俯瞰表示。

|表示|意味|
|---|---|
|白／黒／灰色のmap|選択したcost。白=0、黒=100、unknownはRVizのmap色|
|青線|初期化後の位置・向きから作った直進参照。GNSS経路ではなくodom上の固定線|
|赤〜黄〜緑の線|採用されたMPPI予測速度。低速ほど赤、高速ほど緑。白文字は時刻とm/s|
|RobotModel|URDFの原寸モデル|
|Color image|カメラのカラー画像|

直進参照は車体が向きを変えるたびに追従回転させず、odom上に固定する。
GUIの「現在の位置・向きから直進参照を再設定」で作り直せる。
modeの切替だけでは参照を作り直さない。

線幅は参照8cm・ローカルパス12cm。RVizのDisplaysで対象Pathを開き、
Line Style=Billboards、Line Width（m）で変更できる。Linesでは幅指定が効かない。

設定GUIで以下を変更できる。

速度と内部予測窓の初期値は`pm_config/config/planner.yaml`に従う。
現在は目標1m/s・25 steps・dt=0.1秒（2.5秒先）。GUIに実効値と実際の軌道長を表示する。
可視化timerの2Hzとは独立である。
footprintも正本の`footprint_radius_m`と`footprint_margin_m`を使用する。
現在は車体半径0.3m＋追加余裕0.1m＝膨張半径0.4m。セル幅を更に足さない。
GUIで取得した実効値を表示し、両項目を独立に変更できる。余裕は0も指定できる。

- map一辺、セル幅、depth上下限、pixel stride。
- obstacle／slope／roughness／stepの閾値。
- 解除窓N、最低セル点数、点数比、最小z許容差。
- obstacleのみ／obstacle＋terrainの選択。
- 直進参照の長さ、MPPIの目標速度、車体半径・footprint追加余裕。

「適用」は認識条件を原子的に変更し、mapを消去して新しい画像から再生成する。
「cost切替・予測設定だけ適用」はmapを保持したまま表示とMPPI入力を切り替える。
認識とpreviewは別nodeなので、二つをまとめたtransactionではない。応答結果を画面に表示する。
「認識mapだけ消去」は自己位置原点を変えずに観測結果だけを消去する。

変更は実行中の値に反映する。正本YAMLは自動で書き換えない。
「画面の認識設定を別YAMLへ保存」で保存したものは次回`params_file`へ指定できる。
保存対象は認識設定であり、JoyやMPPIの設定は含まない。

```bash
ros2 launch pm_bringup perception_lab.launch.py drive:=true params_file:=/absolute/path/perception_lab.yaml
```

通常の初期値は`pm_config/config/planner_perception.yaml`。
QoSやtopic・TF設定など、購読再生成が必要な項目はGUIの実行中変更対象ではない。

## パスが表示されない場合

unknown・危険・map外はMPPIで通過可能としない。カメラの視野外となる車体直下、
初回観測前、解除後のunknownなどがfootprintへかかると、有効な予測がない場合がある。
GUIに「hazardまたはunknownを避ける候補軌道がありません」等の理由を表示し、
古い予測線を消す。これは故障や自動走行許可を示す表示ではない。
既観測領域へ車体が移動した後に予測可能になる場合がある。
本ツールは未観測の初期footprintを自動的にfreeへ書き換えない。

実測速度が前進MPPIの範囲外（後退・旋回速度超過等）なら、その理由を表示して予測を消す。
手動操縦そのものをこのpreviewが制限するわけではない。
単一の局所最適化であり、長距離の迂回経路を必ず発見する機能でもない。

## 現在の保持・破棄規則

|対象|規則／変更できる値|
|---|---|
|rolling範囲外のセル|破棄。`map_size_x/y`で範囲を変更|
|高点の新観測|初回からobstacle=100。解除窓を高い値から再開|
|障害物の低い再観測|`confirmation_window`枚の窓内の有効高さ平均で解除|
|障害物の欠測・品質不足|欠測として窓を進める。全欠測窓ならunknownへ戻す|
|解除して0にできる条件|現在も有効な低い観測があること。最低点数・点数比・最小z差も確認|
|通常の非危険セル・terrain有効値|新しい有効結果で更新。時間TTLはなくrolling範囲内で保持|
|画像自体の停止／TF不成立|窓を進めない。時間だけで障害物を解除しない|
|GUIから認識条件を変更|mapを全消去|

窓の欠測は高さ0として平均しない。ただし視野外・遮蔽・support不足も欠測となるため、
物体が存在しないことを証明する解除ではない。
「最後の観測から何秒で破棄」という独立したTTLパラメータは現在の新方式にはない。
旧方式の`observation_decay_time`や`feature_max_observation_age`は、新方式の保持TTLではない。
previewには別に受信／stampのtimeout（既定1秒）があるが、これは古い入力でパスを出さないためのもの。

現在は保持方式`body_aligned`、map一辺8m、前方offset=2m。
車体の向きに追従し、前方約6m・後方約2m・左右約4mを保持する。
従来の`odom_aligned`へGUIで戻せる（この方式ではoffsetを適用しない）。
設定変更時はmapをリセットする。実際の保持範囲をRVizで橙色の輪郭として表示する。
出力はodom軸固定の外接矩形なので、旋回でmessageのwidth/heightが変わり、角部分はunknownとなる。
詳細は[perceptionの保持範囲](../pm_perception/README_planner_perception.md#保持範囲と旋回)を参照。
depth上限7mでも、全域のdepthやhazard評価が有効になる保証ではない。
計算量とメモリはセル数・有効投影点数に応じて増える。

## 起動・終了の補足

```bash
# 外部のセンサ・local EKF・認識mapを使い、表示とpreviewだけ起動する
ros2 launch pm_bringup perception_lab.launch.py start_stack:=false

# 画面なしでデータ処理だけ（外部のGUI/RVizを接続する場合）
ros2 launch pm_bringup perception_lab.launch.py gui:=false rviz:=false
```

GUIを閉じるか起動ターミナルでCtrl-Cするとlaunch全体を終了する。
drive=trueの終了時は既存vehicle interfaceの停止処理が働く。部分的な手動killは避ける。
rosbag記録は自動開始しない。CPU負荷の実機評価、実際のカメラ／Wit／ODrive接続、
RVizのOpenGL描画は対象機での確認が必要。

## 主なtopic

|topic|型|
|---|---|
|`/depth_elevation_mapper/planner_costmap`|`pm_msgs/TerrainAssessment`（現行認識の独立二channel）|
|`/perception_preview/selected_costmap`|`nav_msgs/OccupancyGrid`（選択mode）|
|`/perception_preview/reference_path`|`nav_msgs/Path`|
|`/perception_preview/local_path`|`nav_msgs/Path`|
|`/perception_preview/status`|`std_msgs/String`|
|`/odometry/local`|`nav_msgs/Odometry`（既存local EKF/composer）|

GUIは`/depth_elevation_mapper/set_parameters_atomically`と
`/perception_preview/set_parameters_atomically`を使用する。
群の抽出・追跡方式は今回追加していない。現在のセル単位の認識・解除方式を表示するツールである。

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
