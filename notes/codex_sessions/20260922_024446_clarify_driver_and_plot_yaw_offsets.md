User prompt

実機をいま動かせないので、地磁気校正や実測はまた後日とさせてください。
ちなみにいま初期補正としてyawを入れていますが、これはwitmotion IMUのドライバ側での-90deg、その後の解析用プロットスクリプトでの+90deg両方の補正ともゼロとした場合の値を補正に使っているということですか？
今回の解析でプロットした図を見ると、IMUドライバ側の補正だけ効いている状態に見えます、すなわち、反時計回りに90〜130degくらい回転させれば、local odometryとGNSSが一致しそうです。

Codex final response

はい、後日の実機校正で問題ありません。

そして御認識の後半が正しいです。今回の初期化・GNSS重畳で使ったyawは、「ドライバの`-90°`は既に効いているが、解析スクリプトの`+90°`は一切使っていない」値です。

正確には以下です。

```text
HWT905素のyaw
  → hwt905_rs485_driver: yaw_ros = yaw_imu - 90°
  → /wit/imu
  → heading_initializer: yaw + yaw_correction_radians
  → 今回は yaw_correction_radians = 0°
```

ドライバ内の`-90°`は[hwt905_rs485_driver/imu_node.py](/home/kazuho/ros2_ws/src/hwt905_rs485_driver/hwt905_rs485_driver/imu_node.py:155)で実装されています。一方、従来の[plot_bag_trajectories.py](/home/kazuho/Projects/patasmonkey_ws/src/pm_evaluation/pm_evaluation/cli/plot_bag_trajectories.py:903)の`+90°`は、表示用にドライバの`-90°`を実質的に相殺する補正です。地理・磁気の校正値ではありません。

今回追加した[GNSS・OSM重畳CLI](/home/kazuho/Projects/patasmonkey_ws/src/pm_evaluation/pm_evaluation/cli/plot_initialized_odometry_overlay.py)は`align_local_trajectory_to_north()`を呼んでおらず、GNSSへの平行移動だけを行います。`+90°`、`pi/2`、任意の描画回転は適用していません。従って、重畳図はまさに「ドライバの-90°だけが入った状態」です。

したがって図から読んだ「さらに反時計回りに約+90〜+130°」は、将来`heading_initializer.yaml`の`yaw_correction_radians`へ加える候補です。

- `+90°`なら、ドライバの`-90°`を打ち消し、HWT905素のyaw相当へ戻す。
- `+130°`なら、素のyaw相当に対してさらに`+40°`を加える。

ただし、この90〜130°を今すぐ固定値として採用するのは避けるべきです。今回のGNSSにはNO FIX/FLOATが含まれ、wheel/VIO由来の軌跡ドリフトも重なっているため、見た目の一致だけでは磁気偏角・取付角・磁気外乱を分離できません。実機で停止時再現性と十分な速度の直進GNSS courseを使って値と符号を決めるまでは、`yaw_correction_radians: 0.0`を維持します。

実機校正の際は、プロット側の+90°を使わず、この新しい重畳CLIで確認します。そこで求めた値を運用時の`yaw_correction_radians`へ入れる、という一方向の補正にします。
