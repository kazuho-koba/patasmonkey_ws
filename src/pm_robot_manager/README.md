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
