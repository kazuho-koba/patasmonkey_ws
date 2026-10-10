# pm_gui

Patasmonkey UGVのROS 2 Foxy用操縦・監視GUIです。Qt画面とROS callbackを分離し、ROS通信が途切れてもQt event loopをcallback待ちで止めない構成です。

監視画面はタブを使わない一画面構成です。PATASMONKEY / OPERATOR CONSOLEのタイトルは画面最上段に固定します。起動時は左を画面幅の約3/5にし、上段のOAK-Dカラー画像へ左列の約3/4を割り当てます。画像は受信frameのaspect ratioを保って表示します。左下は姿勢3Dとjoystick状態を左右に並べます。右上約2/3はmap軌跡・GNSS位置、右下はRobot Manager状態とRobot Coreの状態連動ボタン、mission/debugそれぞれのrosbag記録ボタンをコンパクトに表示します。右下はscrollせず操作できます。各境界はsplitterで調整できます。

## 起動

通常のカメラ表示は `/oak/preview/image/compressed`（CompressedImage）を
BEST_EFFORT・履歴1で受信します。Jetsonの `oak_preview_node` が最大5 Hz、幅160 px、
JPEG quality50に変換し、OAK本体とは別プロセスで配信します。元の画像はVIO／mapper／
bag向けに保持されるため、遠隔表示目的で `/oak/color/image_raw` を購読しないでください。
新しい設定を使うにはGUIを再起動してください。

`camera.transport: compressed` と `camera.topic` でJPEG入力を指定します。
既存bagを直接確認するmock設定などでは `camera.transport: raw`（省略時もraw）と
原画像topicを指定できます。GUIのON/OFFはsubscriberの生成／破棄を行います。

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

OAK-D camera displayをONにした間だけ、設定されたcamera topicをBEST_EFFORT、queue depth 1でsubscribeします。通常は表示専用JPEG、bag再生用mock設定は原画像です。このQoSはBEST_EFFORT publisherとRELIABLE publisherの両方から受信できます。OFFではGUI側subscriptionを破棄します。camera driverや他nodeのpipelineには操作を送りません。画面上部のcamera statusには状態、topic、最終受信時間、callback数、cv_bridge変換数・エラー数、Qt描画更新数を表示します。

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

## JOYSTICKタイルのモーターイネーブル

タイルを左右に分割し、左に走行許可の円形ボタン、右に従来のスティックと速度を表示する。
左の円は右のスティック外円と同径。OFFは緑のSTART、走行用launch稼働中は赤のSTOP。
遷移中は黄色のWAIT、Manager未対応・未接続では操作不可とする。
赤のSTOPは走行許可ONを意味し、実際の回転中を意味しない。
右のDISABLED/ENABLED/TURBOは従来どおりjoystickのdeadman/turboボタンの状態であり、
左の走行許可とは独立して表示する。

STARTは確認ダイアログ後、Managerのvehicle/start serviceへ依頼する。
STOPは確認を挟まずvehicle/stopを依頼する。GUIから速度指令は送信しない。
ODrive接続待ち、中立指令待ち、緊急停止、status STALEも左下に表示する。
Core起動時は走行許可OFF。GUI起動時はManagerが管理している現在の状態を表示し、
GUI終了だけでは走行用launchを止めない。

実機側のManager、bringup、teleop、control、systemd/sudoersの更新が必要。
旧Managerでは左ボタンを操作不可にする。Core/bag操作の既存interfaceは維持する。
単体の開発表示は、従来の`gui_mock.yaml`で同じSTART/STOP遷移を利用できる。

```bash
PM_GUI_ROS_DOMAIN_ID=227 \
PM_GUI_CONFIG=/workspaces/patasmonkey_ws/src/pm_gui/config/gui_mock.yaml \
./scripts/pm_gui_container.sh
```

mockのCoreを先に起動すると、左のSTARTを操作できる。
mockは実機やモーターを操作せず、Core再起動後は走行許可OFFへ戻る。

## BL1860B並列電源の残量推定

左下のBATTERYパネルは`pm_msgs/msg/MotorState`の`vbus_voltage`と`ibus_a`を使う。
購読先は`topics.motor_state`（既定`/motor_state`）、QoSはsensor data。
電池外形・残量バー・概算%・実測V・電圧警告をセットで表示する。
下段は左から電池、車体姿勢、走行許可/joystickを1:3:4の初期比率で配置し、
splitterによる幅調整とwindowのサイズ変更も維持する。

### 停止中の電圧基準

両軸の指令と実回転がゼロ付近の場合を停止とし、連続5秒後から電圧基準を更新する。
電圧は停止区間だけで指数平滑化（既定の時定数5秒）し、YAMLの電圧/SOC表を区間線形補間する。
走行中の負荷電圧はこの基準へ混ぜない。初回の停止中受信は暫定電圧基準として表示する。
停止直後の5秒間は直前の推定を保ち、整定後は電圧推定へ再校正するので、停止後に%が補正される場合がある。
5秒は運用上の整定時間であり、厳密な無負荷OCVや電気化学的平衡を保証しない。

マキタのBL1860B資料は18V、6.0AhのLi-ion電池と説明している。
2個並列の共通電源は18V系であり、公称容量の合計を12Ahとして扱う。
各電池の残量や接続状態は、共通busのV/Aだけでは区別できない。

BL1860Bのメーカー校正済み電圧/SOC曲線や放電停止電圧は公開資料から確認できなかった。
5直列Li-ionを仮定した汎用の目安曲線であり、メーカー実測の校正表ではない。
参考セルの公称3.6V・充電終止4.2Vを参照しているが、そのセルがBL1860Bに搭載されているとは主張しない。
16〜21Vを0〜100%へ割り当て、途中の点もYAMLで変更できる暫定的な近似である。
0%は物理的な完全放電を意味しない。負荷・温度・劣化・配線損失による誤差がある。

### 走行中の正のIbus積算

停止中に得た最後の残量を基点に、次の式で走行消費分を引く。

```text
容量Ah = capacity_ah_per_pack × parallel_packs
消費Ah = max(ibus_a, 0) × サンプル間隔秒 / 3600
残量% = clamp(直前の残量% − 100 × 消費Ah / 容量Ah, 0, 100)
```

直近電流をそのサンプル区間の代表とする矩形積分で、低い走行電圧から%を再計算しない。
負のIbusはゼロとして扱い、回生で残量を増やさない。ODrive全体のbus電流なので、左右分として二重加算しない。
停止整定後に基準電圧を更新するたび、基準以降の走行消費Ahをリセットする。
実測電圧、基準電圧、積算Ah、設定容量、Ibusは表示とtooltipで確認できる。

積分には`MotorState.stamp`を使用し、rosbagの再生速度で消費Ahが変わらないようにする。
stampがゼロで使えない場合だけmonotonic受信時刻を使用する。
基準時計の切替、データ時刻の逆行、データ/受信間隔がstale timeoutを超える場合は積算を無効化する。
通信断中の消費や電池交換を推測せず、走行中は「停止基準待ち」で%を未取得に戻す。
GUIを走行中に初めて起動した場合も同様。停止中に再基準を得れば復帰する。
走行中のIbus欠落・NaN・無限大も未計測の区間をゼロ消費と見なさず、停止基準の再取得を要求する。

### YAML設定と注意表示

- `capacity_ah_per_pack`: 電池1個の実効容量Ah。既定6.0。劣化時は実測容量に調整できる。
- `parallel_packs`: 並列個数。既定2、正の整数。
- `voltage_soc_curve`: 昇順の`[共通bus電圧V, 残量%]`。端点0/100%、区間線形補間。
- `rest_settle_sec`: 停止判定が連続してから電圧へ再校正するまでの時間。既定5秒。
- `smoothing_time_constant_sec`: 停止電圧の指数平滑化の時定数。既定5秒。
- `stationary_command_rps`: 両軸の指令が既定0.001rps以下であることを要求。
- `stationary_velocity_rps`: 両軸の実回転が既定0.1rps以下であることも要求。
  どちらもMotorState内のモーター軸単位。非有限値は停止と判断しない。
- `low_voltage_v`: 瞬時17V以下で黄色の「低電圧 / 交換目安」。
- `critical_voltage_v`: 瞬時16V以下で赤の「使用中断目安」。
- `high_voltage_v`: 瞬時21.3V超で赤の「電圧範囲外」。
- `stale_timeout_sec`: 既定3秒。受信停止後は残量・電圧を`--`へ戻してSTALEを表示。

これらの閾値はメーカー保証の安全使用範囲ではなく、GUIの注意表示用の暫定値。
警告は瞬時電圧から計算し、残量積算や平滑化で低下を隠さない。%が高くても電圧警告を優先する。
0V・非有限値は無効とし、空電池の0%と区別する。
表示はモーター出力を変更しない。各セルの電圧・電池温度・保護回路動作は確認していない。
マキタのSTAR Protectionには工具との通信があり、ODrive給電で同じ保護が働くとは仮定しない。
ODriveのIbusは推定DC電流であり、他の電池負荷は含まない。これらも推定誤差になる。

### 開発PCでの反映

`pm_msgs`と`pm_gui`を開発コンテナでビルドしてから、overlayをsourceしてGUIを再起動する。
新版MotorStateの`ibus_a`を公開するpublisherが必要。旧message定義のbagではこのフィールドを取得できない。
走行許可OFFやODrive未接続でMotorState配信が止まると、パネルは待受/STALEになる。
GUI表示のために制御nodeを自動起動することはない。

### 参照資料

- [マキタBL1860B仕様](https://makitatools.com/products/details/BL1860B/)
- [Murataの参考セル仕様](https://www.murata.com/-/media/webrenewal/products/batteries/cylindrical/datasheet/us18650vtc5-product-datasheet.ashx?cvid=20250324010000000000&la=en)
- [電圧推定の負荷依存性](https://www.ti.com/document-viewer/lit/html/SSZT786/GUID-5DD8C3D9-BCBE-467B-90E5-FAA672AAC15D)
- [ODrive firmware0.5.6のDC bus電流定義](https://raw.githubusercontent.com/odriverobotics/ODrive/fw-v0.5.6/Firmware/odrive-interface.yaml)
