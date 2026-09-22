User prompt:
他のセンサも起動、オドメトリ推定も最新版を使用、必要なbagファイルも全て記録するという条件（pm_bag_global_localization.launch.pyに加えてphase2までの処理と同義でしょうか？）で計測を行ってください。
jerkを加えてVOを開始しなければならないので、jerkが必要なタイミングで私に教えてください。

jerkしました

Codex final response:
full-stack実機計測を実施し、終了済みです。車両への速度指令・キャリブレーションは送信していません。

結論として、元の`pm_bag_global_localization.launch.py`は最新localizationを`localization_mode:=separated_offroad`で起動できますが、Stage 2 mapper自体とその出力のbag記録は含んでいませんでした。今回、次を追加してJetsonへ反映・ビルドしました。

- `depth_elevation_mapper_node`をOAK-Dと同じ起動段階で開始
- elevation / relative elevation / variance / count / age / obstacleの6 debug OccupancyGridを同じbagへ記録
- `pm_bringup`に`pm_perception`の実行時依存を追加

実行構成は、全センサ、OpenVINS、WIT IMU、GNSS系、最新の`separated_offroad` localizer、Stage 2 mapper、rosbag recordです。OpenVINSはjerk後に`successful initialization`を確認しました。

記録bagは次です。

```text
/home/nvidia/patasmonkey_ws/bags/rosbag2_2026_09_23-00_15_00_fullstack_stage2
```

記録時間は282.83秒、サイズは14.2 GiBです。以下を確認しました。

- `/oak/depth/image_raw`: 2710件
- `/oak/depth/camera_info`: 2708件
- `/wit/imu`: 27535件
- `/vio/odometry`: 29165件
- `/odometry/local`: 13920件
- `/tf_static`: 5644件
- Stage 2の全6 debug layer: 各442件

Jetson full-stack負荷（60秒計測）:

| 指標 | 実測 |
|---|---:|
| mapper平均CPU | 58.6%（1 core換算） |
| mapper最大CPU | 63.0% |
| mapper最大RSS | 71.9 MiB |
| mapper処理時間 | 約25.4〜29.1 ms/frame（定常） |
| mapper最大処理時間 | 132.3 ms/frame |
| 有効depth sample | 約11,500 points/frame |
| 融合セル | 約370〜415 cells/frame |
| TF drop | 0 |
| Jetson全core使用率 | 概ね70〜80% |
| Jetson RAM | 約3.35 GiB / 14.9 GiB |
| CPU温度 | 約62 °C |
| GPU演算使用率 | GR3D 0% |

定常時は10 Hzの100 ms予算内で動作していますが、最大132 msのspikeがあり、mapperの`rate_drops`は63でした。TF timeoutによる破綻はなく、全stackでCPU余裕は大きくありません。Phase 3は実装可能ですが、局所計算をNumPy vectorizedな固定配列処理に保ち、Phase 3後に同じfull-stack条件で再測定する必要があります。実機runtimeでは不要なdebug layerを無効化して比較することも有効です。

重要な制約があります。vehicle interfaceはODrive接続失敗を繰り返し、`/wheel/odometry`は0件でした。したがって今回の評価はOAK-D・OpenVINS・WIT IMU・最新localization・perception・bag記録の同時負荷を測れていますが、wheel入力まで健全な完全な走行stack評価ではありません。ODrive接続を復旧した後に、wheel odometryを含む再計測を1回行うべきです。

Jetson上の計測ログは`/tmp/pm_fullstack_stage2_perf_20260923_001619/`、launchログは`/tmp/pm_fullstack_stage2_live.log`に残しています。停止時にbagの`metadata.yaml`生成も確認済みです。
