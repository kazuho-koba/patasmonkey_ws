# 5条件EKFと初期heading offset検証

対象bagをFoxy robot_localization 3.1.2で実時間再生した。第5条件
`wheel_gyro_vio`はwheel vx、`/wit/imu` wz、`/vio/odometry` vx/vyのみを使用する。
Wit orientation、wheel wz、VIO poseは使用しない。VIO pose x/y/yawを使った予備試行は
数値発散したため採用せず、元のlocal EKFと同じVIO twistを使う条件に修正した。

## +90度表示での比較

| 条件 | 始終点距離 [m] | GNSS RMSE [m] | 最終GNSS距離 [m] |
| --- | ---: | ---: | ---: |
| wheel vx + wheel wz | 42.28 | 34.75 | 56.36 |
| wheel vx + Wit yaw/wz | 53.73 | 52.74 | 69.91 |
| wheel vx + Wit wz | **0.85** | 17.26 | 15.41 |
| wheel vx + Wit wz + VIO vx/vy | 6.28 | 18.10 | 22.49 |
| VIO pose | 1.52 | 16.44 | 17.44 |
| 記録済みlocal EKF | 56.36 | 52.05 | 72.52 |

第5条件は往復形状を保ち、Wit orientation使用条件より大幅に改善した。ただしこのbagでは
VIO速度を加えるとWit wzだけの条件より閉路性が悪化した。VIOの遅延、横速度、共分散、
wheel vxとの重複観測の重み付けを次に検証する必要がある。

## 理論上のoffset

- ROSと地図はENU、yaw 0が東、反時計回りが正。
- URDFのbase_link→wit_imu_link yawは0度。
- 現行HWT905ドライバはセンサyawから90度を引いて公開する。
- plotの+90度はこのドライバ処理を相殺している。

したがって現在のコードを前提に生センサyawへ戻すだけなら+90度。IMUドライバが
正しいROS ENU yawを公開する設計ならplot側は0度である。どちらも真北との一致を
保証しない。Wit yawが磁北基準なら場所・日時の磁気偏角補正が追加で必要だが、
今回の磁気yawには方位依存の数十度の歪みがあるため、一定角だけでは補正できない。

## 今回bagのGNSS整合offset

offsetは「記録された `/wit/imu` yawへplot時に加える角度」と定義した。

- GNSS ENU速度ベクトル主推定: **+119.70度**（91点）
- 速度の閾値感度: +118.60～+119.83度
- 単純bootstrap 95%: +116.89～+122.61度
- GNSS 5～30秒位置差: +122.23～+122.48度
- 全位置Procrustes: +116.90度

このbagでは暫定値を約+120度とする。+90度より約30度反時計回りである。
比較開始時の公開Wit yawは159.36度なので、+90度ではENU yaw 249.36度
（北基準方位角200.64度）、+119.70度ではENU yaw 279.06度
（北基準方位角170.94度）となる。
ただしRTK FIXは0件、NavSatFix水平合成sigma中央値は10.38 mであり、bootstrapは
系統誤差を表さない。他bag、とくにRTK FIXまたは高品質なGNSS速度で再検証が必要。

`heading_offset_analysis.png/json`に目的関数、時系列残差、閾値感度を保存した。
`comparison_*_gnss_offset_119p7.png`は+119.7022度で地図へ再描画した結果である。
