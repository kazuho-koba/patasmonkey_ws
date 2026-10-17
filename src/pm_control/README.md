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
