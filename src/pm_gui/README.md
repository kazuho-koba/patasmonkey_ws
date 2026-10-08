# pm_gui

Patasmonkey UGVのROS 2 Foxy用操縦・監視GUIです。Qt画面とROS callbackを分離し、ROS通信が途切れてもQt event loopをcallback待ちで止めない構成です。

監視画面はタブを使わない一画面構成です。PATASMONKEY / OPERATOR CONSOLEのタイトルは画面最上段に固定します。起動時は左を画面幅の約3/5にし、上段のOAK-Dカラー画像へ左列の約3/4を割り当てます。画像は受信frameのaspect ratioを保って表示します。左下は姿勢3Dとjoystick状態を左右に並べます。右上約2/3はmap軌跡・GNSS位置、右下はRobot Manager状態とRobot Coreの状態連動ボタン、mission/debugそれぞれのrosbag記録ボタンをコンパクトに表示します。右下はscrollせず操作できます。各境界はsplitterで調整できます。

## 起動

Foxy container内でworkspaceをbuildし、overlayをsourceした後に起動します。

```bash
ros2 launch pm_gui operator_console.launch.py
```

実機なしのmanager mockは次のように起動できます。mock manager操作とrosbag replay topic監視を同じwindowで利用できます。

```bash
ros2 launch pm_gui operator_console.launch.py \
  config:=/workspaces/patasmonkey_ws/src/pm_gui/config/gui_mock.yaml
```

Jetson上のROS domainと分離してbag replayを試す場合は、GUIとbag playerの両方に同じdomainを指定します。GUIはdomain 227で起動します。

```bash
PM_GUI_ROS_DOMAIN_ID=227 \
PM_GUI_CONFIG=/workspaces/patasmonkey_ws/src/pm_gui/config/gui_mock.yaml \
./scripts/pm_gui_container.sh
```

別のhost terminalでは、GUI wrapperと共通のcontainer・UID・domain設定を使うbag wrapperを実行します。利用者が`--user 1000:1000`を毎回指定する必要はありません。

```bash
PM_GUI_ROS_DOMAIN_ID=227 \
./scripts/pm_gui_bag_play_container.sh \
  /workspaces/patasmonkey_ws/bags/rosbag2_2026_08_10-18_42_30 \
  /oak/color/image_raw \
  /odometry/global \
  /cmd_vel_joy \
  /pm/joy \
  /fix \
  /navpvt
```

この例はカメラに加えて自己位置、操縦指令、joystick、GNSSの表示用topicを再生します。`/oak/color/image_raw`だけを指定すると、他の表示項目に入力が届かず待受状態になります。topicを省略するとbag内の全topicを再生します。wrapperのdomain指定を省略すると、container起動時のROS domainをそのまま使います。GUIもbag playerも同じdomainで起動してください。

## 表示interface

初期設定は`config/gui.yaml`にあります。mapは有効な`/fix`のWGS84緯度経度を中心にし、`/fix`の履歴を地図上へ描きます。車両headingは`/odometry/global`から重ねます。map座標のx/yを緯度経度へ変換しません。自己位置のmap-frame x/y/zは数値で併記し、GNSS lat/lonと区別します。地図画像内のスケールバーは中心緯度とzoomから地上距離を計算し、拡大縮小やpan、位置更新に合わせて自動更新します。地図上でマウスホイールを回すか、地図画像右上に重ねた「ー／＋」ボタンを押すと、縮尺を1段階ずつ変更できます（z1〜z19）。地図画像上を左クリックしたままドラッグするとパンできます。手動パン中は地図中心を固定し、車両マーカーはGNSS位置に合わせて動きます。画像右上の「追従」ボタンで車両位置を中央に戻せます。

GNSS品質は`/navpvt`の`FLAGS_GNSS_FIX_OK`、`fix_type`、carrier phase flagsを使い、`/navpvt`が利用できない場合だけ`/fix`のstatusでGNSS/NO FIXを表示します。joystick画面は`/pm/joy`のaxis/buttonを監視し、teleopへpublishしません。axis ID、enable/turbo button ID、deadzoneは起動時に`pm_teleop`の`teleop_twist_joy.yaml`と`joy_params.yaml`から読み込み、円内のstick位置とmode色に反映します。yaw axisをX、linear X axisをYとして使い、X正は右、Y正は上に表示します。試験時の左入力との対応に合わせ、`joystick.axis_display_sign.x: -1.0`でGUI上のX表示だけ符号反転します。ROS teleopへ渡す値は変更しません。

姿勢パネルは`pm_description/urdf/pm.urdf`と同packageのSTL meshを使い、roll/pitch/yawを3D描画します。シャシはダークグリーン、wheel armは白、タイヤは黒で表示します。ENU水平基準面はbody中心の高さを通り、yawだけ車体に追従します。roll/pitchには追従せず、モデルmeshと同じ奥行き順で描くためbodyに部分的に遮蔽されます。黄色い前方矢印の根元は、URDFの`body` mesh前端中央に固定します。モデルはbase_link基準で、前方はROS `+X`です。ENU規約のROS yawは`0°=東`、正方向が反時計回りなので、画面では東を右向きとして描画します。表示用の追加yaw offsetは適用しません。描画倍率は`attitude.pixels_per_meter`で固定します。初期window寸法は`ui.window_size`を使い、primary monitorの作業領域に収まるよう調整します。通常設定ではresizeと最大化が可能です。固定したい場合は`ui.fixed_window_size: true`にします。

現行localization設定の`heading_initializer.yaml`にある`yaw_correction_radians`は`0.0`です。関連READMEではこれは実機校正済み値ではないとされています。GUIの見た目だけでlocalization補正値を変更せず、姿勢表示は`/odometry/global`が出す姿勢をそのまま使用します。必要な場合は`attitude.urdf_path`で別のURDFファイルを指定できます。

OAK-D camera displayをONにした間だけ`/oak/color/image_raw`をBEST_EFFORT、queue depth 1でsubscribeします。このQoSはBEST_EFFORT publisherとRELIABLE publisherの両方から受信できます。OFFではGUI側subscriptionを破棄します。camera driverや他nodeのpipelineには操作を送りません。画面上部のcamera statusには状態、topic、最終受信時間、callback数、cv_bridge変換数・エラー数、Qt描画更新数を表示します。

Docker imageには`fonts-noto-cjk`を含め、Qtの標準fontにNoto Sans CJK JPを設定します。

## rosbag

mission/debugの記録開始・停止はLaptop上ではなく、JetsonのRobot Manager経由で各bag launch unitへ依頼します。Mission bagの自動開始はJetsonのboot時のみです。GUIからCoreを起動してもMission/DebugはOFFのままで、各記録ボタンから個別に開始します。状態、Jetson上の出力先、経過時間、Jetson側の空き容量をManager statusから表示します。GUIを閉じてもCoreとbagは継続します。

bag停止ではManagerがrecorderの停止serviceへ要求を送り、recorderがSIGINTをrosbagへ転送します。recorderは`metadata.yaml`の存在と`ros2 bag info`の成功を`completion.json`へ記録します。Managerは保存検証の後にsystemd bag unitを停止し、Core停止要求時はMission/Debug両方の確認後にCore unitを停止します。確認できない場合はbag unitを停止せず、Core停止も中断します。systemdからbag unitを直接停止した場合も、ExecStop helperが同じsystemd invocation IDのstatus fileとcompletionを照合し、保存確認後にだけunit停止へ進みます。検証失敗時は待機を続け、`TimeoutStopSec=infinity`と`SendSIGKILL=no`でsystemdの強制killを無効にします。

Robot ManagerのROS interface、systemd unitの関係、Jetsonへの配置手順は`pm_robot_manager` package内のREADMEを参照してください。

## 現段階の表示範囲

地図は設定されたtile URLから現在viewport内のXYZ tileだけを非同期取得し、cache headersに従って保存します。OSM標準tileを使うときはUser-Agentと画面内の attributionを設定し、先読みやエリア一括downloadは行いません。online時に実際に表示したtileは`~/.cache/pm_gui/osm`へ保存され、通信断後も再利用します。追加のoffline XYZ tileは`map.offline_tiles_dir`で指定できます。cacheにもoffline tileにも対象地域がない場合は、GNSS軌跡と緯度経度gridを表示します。初回から通信なしで道路画像を出すには、利用条件に沿った地域のoffline XYZ tileを同directoryへ用意してください。Vehicle ENABLE、emergency stop、joystick enableの操作機能は含みません。

## Mission Planner

タイトル右のMISSION PLANNERから独立した`pm_mission`を起動できます。
経路編集・保存・UGVへの受領確認を行い、走行は開始しません。
設定の`mission_planner.config`に任意のPlanner YAMLを指定できます。
地図tile providerは`pm_ui_common`へ共通化し、既存importは維持しています。

## 地図の拡大上限

map.tile_max_zoomは取得tileの上限（19）、map.max_zoomは表示上限（26）です。
Z19を超えた表示は取得済みtileの画像拡大です。車両・軌跡・pan・scale barは
表示zoomを使います。画像拡大自体は地図の位置精度を向上させません。

## 現在速度表示

Joystickの円下にホイールodometryのvx × 3.6をkm/hで表示します。
20ポイントでmodeと同じ色です。後退は負値、未受信・STALE・非有限値は
`--.- km/h`とします。sourceはtopics.wheel_odometry（既定`/wheel/odometry`）、
nav_msgs/Odometry.twist.twist.linear.xです。sensor QoSで購読します。
# Wit磁気コンパス表示

姿勢パネルの`Odom yaw`は`topics.odometry`（既定`/odometry/global`）の
quaternionから求める相対姿勢であり、東西南北とは解釈しません。
磁気方位は別欄で`/wit/mag`と`/wit/imu`から計算します。北0度・時計回り
（東90度、南180度、西270度）です。未受信やstaleは待受表示、無効値は理由を
表示し、欠測時に0度へ置き換えません。地図矢印と3Dモデルのyawは従来のodomを
維持し、コンパス値をEKF・地図座標変換・走行制御へは入力しません。

`/wit/imu`のroll/pitchだけで磁気ベクトルの傾斜を補償し、orientation yawは
使いません。同じframeの新鮮なIMU姿勢が必要です。取付軸はx前方/y左/z上を
仮定します。Witドライバは現状`/wit/mag`にraw register値を格納しているため、
磁場のTesla単位には依存せず方向比だけを使います。ドライバのyaw -90度補正は
この磁気方位計算には入りません。取付軸が異なる場合は正しいframeへの軸変換を
先に行ってください（offsetだけで軸反転や3D取付姿勢を直すことはできません）。

`gui.yaml`の`compass`でbias（磁気データと同じ単位）、軸別scale、方位offset、
偏角を設定できます。偏角0なら磁北基準で、真北への補正は行いません。
初期値は未校正・参考値と表示します。`calibrated=true`は補正設定済みという
表示の切替にすぎず、品質を自動検証しません。一般のsoft-iron行列や磁気外乱の
検出は未実装です。車体・モータ・電源配線の磁気影響があるため、実機で複数方向
への回転と基準方位を用いた校正が必要です。
