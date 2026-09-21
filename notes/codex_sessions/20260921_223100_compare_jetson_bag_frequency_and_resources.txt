User prompt

途中で指示が変わってしまったようです。
pm_evaluationの内容は全てjetsonへデプロイ・ビルドまで行いました（sourceはしていません）
改めて、前回bagファイルと今回bagファイルで各トピックの周波数とマシンパワー余裕度の比較をしてください。

Codex final response

比較しました。対象は次の2試行です。

- 変更前: `rosbag2_2026_09_21-19_31_30_track21_live`（210.0秒、9.4 GB）
- 変更後: `rosbag2_2026_09_21-22_06_56_track21_queue_drain`（146.7秒、7.1 GB）

Jetsonには `pm_evaluation` の新しいCLIがデプロイ・ビルド済みであることを確認しました。今回は `/opt/ros/foxy` → `~/ros2_ws` → `~/patasmonkey_ws` の順でsourceして実行しています。

| トピック | 変更前 | 変更後 | 評価 |
|---|---:|---:|---|
| `/oak/stereo/left/image_raw` | 19.082 Hz | 19.998 Hz | ほぼ20 Hzへ回復 |
| `/oak/stereo/right/image_raw` | 19.083 Hz | 19.998 Hz | ほぼ20 Hzへ回復 |
| `/oak/color/image_raw` | 9.547 Hz | 10.016 Hz | 期待どおり10 Hz |
| `/oak/depth/image_raw` | 9.530 Hz | 10.000 Hz | 期待どおり10 Hz |
| `/oak/imu/data` | 121.259 Hz | 126.483 Hz | IMU設定（125 Hz）近傍 |
| `/wit/imu` | 99.587 Hz | 99.702 Hz | 約100 Hzで安定 |
| `/wheel/odometry` | 49.640 Hz | 49.767 Hz | 約50 Hzで安定 |
| `/ov_msckf/trackhist` | 16.998 Hz | 19.913 Hz | 画像更新と同じ約20 Hzへ改善 |
| `/ov_msckf/poseimu` | 18.787 Hz | 19.990 Hz | 約20 Hzへ改善 |
| `/ov_msckf/odomimu` | 118.971 Hz | 126.133 Hz | IMU伝播と整合 |
| `/vio/odometry` | 119.436 Hz | 126.471 Hz | IMU伝播と整合 |

周波数は各トピックの最初から最後までのメッセージ間隔で算出しました。今回のVIO pose/path/point cloudは、OpenVINS初期化成功後の約103秒区間だけが対象です。

OAK診断のデバイスsequenceも改善しています。変更前は左ステレオ画像で累計183フレームの欠落と、最大約4.6秒の連続欠落がありました。変更後は左画像が欠落0、右画像が2フレームのみでした。従って、`tryGetAll()` によるキューdrainと、保守的に増やしたキュー長（stereo 20、RGBD 10、IMU 250）は、今回の約19 Hzへの低下を解消できています。

マシン負荷については、変更後の記録中171サンプルで、平均コア使用率48.0%、p95 69.1%、最大75.8%、RAM最大3,037 / 14,887 MB、CPU温度最大56.5 C、GR3D使用率最大0%でした。`vmstat` はCPU idle平均47.3%、I/O wait平均0.15%です。変更前のbag記録区間（211サンプル）でも、平均コア使用率は約53.2%、最大83.4%、RAM最大3,061 / 14,887 MB、CPU/GPU温度最大57.0 / 57.5 C、I/O wait最大約1%でした。

両試行ともメモリ逼迫、温度スロットリング、ディスクI/O待機の増大は確認されません。変更後は負荷を大きく増やさず、入力画像とOpenVINSの更新を約20 Hzまで回復できています。

解析結果は各bag内の `topic_frequency.csv` にあります。再利用する場合は以下を実行できます。

```bash
ros2 run pm_evaluation analyze_bag_frequencies <bagディレクトリ> --overwrite
ros2 run pm_evaluation analyze_oak_bag_diagnostics <bagディレクトリ>
ros2 run pm_evaluation summarize_tegrastats <tegrastats.log>
```

なお、停止時にOpenVINSプロセスがexit code 139で終了した記録が1件ありますが、これはlaunch停止時刻と一致し、今回のbag記録中のVIO出力は継続していました。残留ノードは確認されていません。安全な停止処理の観点では、これは別途原因を切り分ける価値があります。
