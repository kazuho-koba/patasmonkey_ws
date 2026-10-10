# pm_robot_manager

Jetson上でRobot Coreとbag recorderのsystemd unitを操作し、ROS 2 service/topicでGUIへ状態を返すpackageです。ManagerはCoreから独立して常駐し、GUI終了やROS client切断だけではCore・bagを停止しません。

## 管理対象

- `start-pm.service` は `pm_bringup pm_core.launch.py` を起動します。
- `pm-mission-bag.service` は `pm_bringup mission_bag.launch.py` を起動します。boot targetからCoreの後に開始し、GUIからのCore起動には連動しません。Core停止時は先にbagの終了処理を行います。
- `pm-debug-bag.service` は `pm_bringup debug_bag.launch.py` を起動します。Core稼働中にGUIから個別に開始できます。
- `pm-robot-manager.service` は常駐し、上記3 unitの状態と操作serviceを提供します。

Core停止serviceは、Debug、Missionの順でrecording stop serviceを呼びます。各recorderがrosbag processへSIGINTを送り、`metadata.yaml`の存在、`ros2 bag info`の終了コード、`completion.json`の`verified`を確認した後にunitが終了します。Managerはbag unitのinactive状態とcompletion fileの検証を待ってからCoreを停止します。検証が失敗したりrecorder応答が失われた場合は、bagへの強制signalを送らずCore停止を中断します。

systemdからbag unitを直接停止する場合も、各unitの`ExecStop`が`stop_bag_wait`を実行します。recorderはJetson上の状態fileに出力先とsystemd invocation IDを記録し、helperは同じIDの`completion.json`だけを採用します。helperは`metadata.yaml`、`ros2 bag info`、completionの検証後に戻ります。検証が失敗した場合もunitを先へ進めず待機するため、管理者が状態を確認するまでCore停止も保留されます。bag unitでは`TimeoutStopSec=infinity`、`SendSIGKILL=no`を指定し、保存確認前のsystemd強制終了を無効にしています。

## ROS interface

| 操作・状態 | Interface |
|---|---|
| Core start / stop / status | `/pm/robot_manager/start`, `/pm/robot_manager/stop`, `/pm/robot_manager/get_status` |
| Mission bag start / stop | `/pm/robot_manager/mission_bag/start`, `/pm/robot_manager/mission_bag/stop` |
| Debug bag start / stop | `/pm/robot_manager/debug_bag/start`, `/pm/robot_manager/debug_bag/stop` |
| recorder内部のgraceful stop | `/pm/robot_manager/{mission,debug}_bag/request_stop` |
| recorder詳細状態 | `/pm/robot_manager/{mission,debug}_bag/status` |
| Manager heartbeatと全状態 | `/pm/robot_manager/status` |

bag recorderの内部stop serviceはrecord_trial processが提供します。外部GUIはManagerのbag stop serviceを呼び、Managerが保存完了まで待ちます。

## Jetsonへの配置

packageをFoxy workspaceへinstallした後、systemd templateとsudoers fragmentを配置します。unitの配置後は`visudo`でsudoers構文を確認してからdaemonをreloadしてください。

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/patasmonkey_ws/install/setup.bash
SYSTEMD_SHARE="$(ros2 pkg prefix pm_robot_manager)/share/pm_robot_manager/systemd"
sudo install -m 0644 "$SYSTEMD_SHARE/start-pm.service" /etc/systemd/system/
sudo install -m 0644 "$SYSTEMD_SHARE/pm-mission-bag.service" /etc/systemd/system/
sudo install -m 0644 "$SYSTEMD_SHARE/pm-debug-bag.service" /etc/systemd/system/
sudo install -m 0644 "$SYSTEMD_SHARE/pm-robot-manager.service" /etc/systemd/system/
sudo install -m 0440 "$SYSTEMD_SHARE/pm-robot-manager.sudoers" /etc/sudoers.d/pm-robot-manager
sudo visudo -cf /etc/sudoers.d/pm-robot-manager
sudo systemctl daemon-reload
sudo systemctl enable start-pm.service pm-robot-manager.service pm-mission-bag.service
```

boot時はmulti-user.targetからManager、Core、Mission bagを開始します。Mission bagにはAfter=start-pm.serviceを指定してCoreの後に起動します。GUIからCoreだけを開始してもMission bagはOFFのままです。Mission/Debugは各記録ボタンで独立して開始します。GUIを閉じても管理対象のlaunchは継続します。Core unitはSIGINTで正常停止し、wrapperのsignal由来の終了コード130/143をSuccessExitStatusとして扱うため、正常停止をfailedと表示しません。

初回配置やunit更新はJetson上で別途実施します。このrepository内のunit template変更だけでは稼働中Jetsonへ反映されません。

### boot時のbag時刻

Mission/Debug unitは`systemd-timesyncd.service`による保存時計の復元後に、
`wait_bag_clock`をExecStartPreで実行します。NTP同期を最大45秒待ち、未同期でも
2020年以降なら復元時計を使用して警告を記録します。2000年など未復元の時計では
録画開始を失敗させます。CoreとManagerはこの待機に依存しません。
`PM_BAG_CLOCK_WAIT_SEC`と`PM_BAG_CLOCK_MIN_YEAR`はunitの環境変数で変更できます。
ネットなしで復元時計そのものが不正な場合、正しい日付は保証できません。
自動bag名はlaunch定義作成時ではなく録画開始段階で生成します。
既存の誤った名前やbag内timestampは変更しません。

ExecStartPreでもROS overlayをsourceしてからconsole scriptを実行します。
boot後に時刻確認処理のpackage解決エラーが出た場合、CoreとManagerを停止せず
`sudo bash ~/patasmonkey_ws/scripts/update_bag_clock_systemd.sh`でbag unitだけ
更新してMission記録を復旧できます。Mission/Debug録画が停止中であることが前提です。

### GUI停止時の保存先未作成bag

Mission/Debug停止時、Managerはrecorderのoutputとsystemd起動IDを照合します。
保存先不在ならunitのprocessをSIGSTOPで一時停止して再確認し、SIGKILLで
対象bag unitだけを取消します。保存先が存在する場合は従来どおり保存検証を待ちます。
保存先不明、起動ID不一致、権限・状態取得エラーでは強制停止しません。
取消は保存成功ではなくCANCELLEDとして扱い、GUI状態はSTOPPEDへ戻します。
この機能には更新したManagerとsudoersのJetson側配置が必要です。

## GUIからの走行許可（モーターイネーブル）

Core、bag、走行許可は別々に操作する。`pm_core.launch.py`は既定で
`use_teleop:=false use_vehicle_interface:=false`となり、joy入力nodeだけを残す。
CoreやMission bagがbootで起動しても、モーター制御は起動しない。
従来の単体teleop/vehicle launchと旧一括launchの既定動作は維持する。

走行許可には独立した`pm-vehicle-control.service`を使う。このunitは
`pm_bringup/pm_drive.launch.py`から速度指令変換とvehicle interfaceを起動し、
既存の`/pm/joy`入力を再利用する。`Install`節を持たず、bootでenableしない。
Coreへの`BindsTo`依存により、Core終了時は走行用unitも停止する。
`Restart=no`により、走行用nodeの異常終了後に自動で再許可しない。

- START: `/pm/robot_manager/vehicle/start` (`std_srvs/srv/Trigger`)
- STOP: `/pm/robot_manager/vehicle/stop` (`std_srvs/srv/Trigger`)
- 接続・出力許可状態: `/pm/vehicle/status` (`std_msgs/msg/String`、JSON)
- Managerの既存status JSONに`vehicle`項目を追加する。
  `state`は走行用launchの状態であり、車両が実際に動いていることを意味しない。
  `detail.connected`はODrive接続、`detail.armed`は中立指令確認後の出力許可、
  `detail.emergency_stop`は緊急停止ラッチを示す。`fresh`は3秒以内の受信を示す。

STARTはCore稼働中だけ受け付け、旧Coreや手動launchの同名走行nodeとの重複を拒否する。
systemdのactiveだけで接続成功と判断せず、新しいODrive接続statusを待つ。
接続待受timeoutでは走行用unitを停止し、遅れて有効になるprocessを残さない。

停止時はsystemdのinactiveに加えて、ROS graphから走行nodeが消えるまで
STOPPINGを維持する（最大`shutdown_timeout_sec`）。異常終了後はDDSに古い
node情報が残ることがあるため、次のSTARTも最大`startup_timeout_sec`まで
消失を待つ。期限内に消えないnodeは重複起動の原因として拒否し、手動launchを
勝手に停止しない。failed状態の管理unitは停止処理をやり直してから確認する。
systemd状態照会のprocess起動失敗・timeoutは最大3回再試行し、開始/停止命令は
自動再発行しない。開始失敗と停止確認失敗は両方をエラーに残す。

ODriveはIDLEでゼロ速度を設定し、接続後に届いた新しい生`/cmd_vel_joy`のゼロ指令を
確認するまでclosed-loopを要求しない。車両制約による変換後のゼロは中立判定に使わない。
USB再接続時にも同じ確認を要求する。停止時は各軸へゼロ速度とIDLEを要求してから終了する。
緊急停止はnode再起動まで保持し、USB再接続で解除しない。
これは非常停止装置や物理的なモーター停止確認を代替するものではない。

GUIの走行STOPはbag操作とは別workerで処理し、bagの保存待ちで受付を遅らせない。
Core STOPは走行用launch、bagの正常保存、Coreの順で停止する。
GUI終了や通信断だけでは走行許可を解除しないため、GUIなしでもjoystick操作を継続できる。

### 後日Jetsonへ反映する際

`pm_control`、`pm_teleop`、`pm_bringup`、`pm_robot_manager`を反映・ビルドする。
Coreとbagと走行用unitを正常停止した後、従来の
`sudo bash scripts/install_robot_manager_systemd.sh`で新unit、Core unit、sudoersを更新する。
このスクリプトはManagerだけを再起動し、走行用unitはenableも起動もしない。
旧unit/drop-inが独自の起動設定を持つ場合は、走行nodeを自動起動していないことを確認する。
GUI側は`pm_gui`を反映する。開発PCのmockでは実機やsystemdを操作しない。
