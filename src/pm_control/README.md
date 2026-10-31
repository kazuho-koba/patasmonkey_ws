# 車両制御と実状態診断

`vehicle_interface_node`は速度指令の制限、ODriveの接続・制御、モータ状態の配信を担当します。
`wheel_odometry_node`は車輪情報からオドメトリを計算します。

## motor enabledの確認

`/pm/vehicle/status`（`std_msgs/String`内のJSON）の`armed`はソフトウェアの走行許可であり、
ODriveの実際の制御状態を保証するものではありません。

受入試験時だけ`motor_state_diagnostics:=true`にすると、既存の2 Hz通知に
`axes`（左・右の順）を追加します。`current_state=8`はCLOSED_LOOP_CONTROLです。
`error`と`input_vel`も同時に読み取り、読み取り失敗は`motor_state_read_error`に記録します。
これは状態の読出しのみで、校正・制御切替・速度指令は行いません。
既定値はfalseで、通常運用では追加USB読み取りをしません。

起動後の切替例:

```bash
ros2 param set /vehicle_interface_node motor_state_diagnostics true
ros2 topic echo /pm/vehicle/status
```

ODriveは制御ノードがUSBを所有するため、別プロセスで`odrive.find_any()`を呼ぶと
インターフェースの取得が競合することがあります。実状態は上記通知から確認します。
診断を終了したらfalseへ戻してください。

**安全上の注意:** 中立確認なしの起動では、既存実装が旧速度入力をゼロへ戻してから
closed-loopへ移行します。ゼロ速度でも保持トルクは発生し得ます。
実機での起動には車体・給電・周辺の安全確認が必要です。

## USB読取り負荷の診断と低頻度feedback（試験用）

`motor_feedback_hz`は左右速度とDCバス電流の読取り頻度です。既定30.0では
従来どおりMotorState配信ごとに実読取りします。0超〜30Hzで起動時に設定でき、
10.0指定では直近の速度・電流を保持してUSBアクセスを減らします。
起動後の`ros2 param set`ではcacheの周期は更新されないため、変更にはnodeの再起動が必要です。
左右位置は常に30Hzで実測し、MotorStateも30Hzで配信します。速度制御指令の
25Hz処理、中立確認、停止・緊急停止処理は変更しません。

低頻度時の速度・電流は位置stampと同時の測定値ではありません。更新間隔は
タイマー遅延も含み、厳密な最大鮮度保証はありません。stampは従来同様、左右
位置取得区間の中点です。電流過渡や速度追従の高頻度解析には既定30Hzを使って
ください。再接続時はcacheを破棄し、読取り失敗を正常値で隠しません。

`motor_polling_timing_csv`の既定値は空文字（診断OFF）です。保存先を指定すると、
位置・速度/電流・電圧・publish各段階の所要時間を最大60000件メモリに保持し、
終了時にCSVへ保存します。既存ファイルは上書きしません。時間は単調時計、
wall_nsは他ログとの区間照合用です。USB応答待ちとCPU実行待ちが各所要時間に
混在するため、このCSVだけでUSBハブの競合とは断定できません。
# 自律指令の手動介入インターロック

自律入力は`/cmd_vel_autonomy`、手動入力は`/cmd_vel_joy`。選択・車両制約・
車輪回転数上限の適用後の車体速度指令を`/cmd_vel`に10 Hzでpublishして記録する。
モータ指令の計算・送信は`motor_command_rate_hz`で指定し、既定10 Hzとする。
10〜25 Hzの有限値を起動時に指定でき、従来条件へ戻す場合は25.0とする。
起動後の変更は拒否するため、設定変更後はnodeを再起動する。
速度・yaw加速度制限のdtは最大0.1秒に制限され、10 Hzで遅延があると設定された
加速度より緩やかになることがある。指令タイムアウトは0.3秒のままで、判定は
指令送信周期で行う。モータ状態取得・publishは設定上30 Hzのままである。
これはモータへの指令相当値であり、接続不成立や通信失敗もあるため実測速度ではない。
停止・入力期限切れ・未armed・緊急停止時はゼロを出す。Simulatorも`/cmd_vel`を購読する。
従来の`/sim_cmd_vel`は廃止したため、外部Simulatorの設定は`/cmd_vel`に変更する。
自律入力のparameter名`cmd_vel_topic`は維持し、既定値を`/cmd_vel_autonomy`とする。
出力の`/cmd_vel`を自律入力に指定すると自己循環を防ぐため起動を拒否する。

vehicle_interface_nodeは起動時に自律入力を無効にする。GUI等から
`/pm/vehicle/set_autonomy_enabled`（`std_srvs/srv/SetBool`）へ`true`を要求して
初めて、その後に新規受信した`/cmd_vel_autonomy`を受け付ける。`false`は無効化する。
このサービスは走行の許可状態を切り替えるため、運用中の車両へ要求する際は注意する。

`/cmd_vel_joy`はゼロ指令を含め、受信するたびに自律許可を解除し、保管済み
自律入力の保存値を消す。Joy指令が途切れても自動復帰しない。直近0.3秒以内にJoy指令を
受信していれば`true`要求は失敗する。再許可後も古い指令は再利用せず、新しい
`/cmd_vel_autonomy`を待つ。GUIからの操作画面は未実装である。
`/pm/vehicle/status`のJSON内`autonomy_enabled`は許可状態であり、実際の動作や
モータ接続・走行準備を意味しない。手動指令の既存優先や緊急停止は別途適用される。
