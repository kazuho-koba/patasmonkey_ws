# pm_gui

Patasmonkey UGVのROS 2 Foxy用操縦・監視GUIです。Qt画面とROS callbackを分離し、ROS通信が途切れてもQt event loopをcallback待ちで止めない構成です。

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
  /oak/color/image_raw
```

topicを省略するとbag内の全topicを再生します。wrapperのdomain指定を省略すると、container起動時のROS domainをそのまま使います。GUIもbag playerも再起動して同じdomainに合わせてください。

## 表示interface

初期設定は`config/gui.yaml`にあります。`/odometry/global`のposeは`map` frameの位置として軌跡を描きます。map座標を緯度経度へ変換せず、実際のWGS84座標は`/fix`から別表示します。yawはROS ENUの角度からcompass bearingへ換算します。

GNSS品質は`/navpvt`の`FLAGS_GNSS_FIX_OK`、`fix_type`、carrier phase flagsを使い、`/navpvt`が利用できない場合だけ`/fix`のstatusでGNSS/NO FIXを表示します。joystick表示は`/pm/joy`と`/cmd_vel_joy`を監視し、teleopへpublishしません。

OAK-D camera displayをONにした間だけ`/oak/color/image_raw`をBEST_EFFORT、queue depth 1でsubscribeします。このQoSはBEST_EFFORT publisherとRELIABLE publisherの両方から受信できます。OFFではGUI側subscriptionを破棄します。camera driverや他nodeのpipelineには操作を送りません。画面上部のcamera statusには状態、topic、最終受信時間、callback数、cv_bridge変換数・エラー数、Qt描画更新数を表示します。

Docker imageには`fonts-noto-cjk`を含め、Qtの標準fontにNoto Sans CJK JPを設定します。

## rosbag

mission/debugのtopic list、出力先、storage、最大bagサイズ、stop timeoutはYAMLで変更できます。GUIが起動したrosbagはSTOP操作でSIGINTを送り、metadata closeを待ちます。timeout後にSIGTERM、最後にSIGKILLへ進みます。Robot Core STOP要求時にはGUI所有bagの停止完了後にmanagerへSTOPを送ります。GUI終了時も所有bagへ正常終了signalを送り、既存Robot Core側の記録processにはsignalを送りません。

既存bringup側のrosbag processはROS graph上の`rosbag2_recorder` nodeを検出して表示します。既存processの出力directoryや開始時刻はROS graphから取得できないため、GUI所有記録と区別して「Robot Core側 recorder検出」と表示します。

## 現段階の表示範囲

localization viewはオフラインでも動作するmap x/y axesとtrajectoryです。WGS84 datumからmap frameへの変換根拠が確認できるまでonline tileは重ねません。Vehicle ENABLE、emergency stop、joystick enableの操作機能は含みません。
