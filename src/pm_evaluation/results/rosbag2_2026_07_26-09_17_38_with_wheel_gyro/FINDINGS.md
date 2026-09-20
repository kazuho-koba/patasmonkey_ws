# Wheel vx + Wit gyro z 比較結果

`source_wheel_gyro.yaml`を追加し、4条件をFoxy robot_localization 3.1.2で
実時間再生した。新条件は`/wheel/odometry`のvxと`/wit/imu`のwzだけを使用し、
Witのorientation、wheel wz、VIOを入力していない。

| 条件 | 共通区間の始終点距離 [m] | GNSSとのRMSE [m] | 最終GNSS距離 [m] |
| --- | ---: | ---: | ---: |
| wheel vx + wheel wz | 42.12 | 34.66 | 56.22 |
| wheel vx + Wit yaw/wz | 53.75 | 52.75 | 69.92 |
| wheel vx + Wit wzのみ | **0.92** | 17.24 | 15.35 |
| VIO pose | 1.52 | 16.38 | 17.44 |
| 記録済みlocal EKF | 56.36 | 52.05 | 72.52 |

共通区間は約190.78秒。Wit wzを直接積分して同じwheel変位に適用した場合の
始終点距離は1.16 mで、wheel_gyro EKFとの軌跡RMSEは0.29 mだった。
したがって新しいEKFは、磁気の影響を受けたWit orientationを除外した意図どおり、
ジャイロが示す往復軌跡を再現している。

GNSSにはRTK FIXがなく水平不確かさも大きいため、GNSSとの値は真値誤差ではない。
また、wzだけでは長時間のジャイロバイアスを絶対方位で修正できない。このbagで
閉路性が良いことは有力な結果だが、長時間・異なる走行条件でも評価が必要。
