User prompt
===========

Jetson側で接続して、pm_bag_global_localization.launch.pyを実行してください。
適当な時間、実行してデータをbagファイル等に記録した上で、そのbagファイルに、ここまでの開発成果である各種情報が、想定通りの周波数で保存されているかどうか、確認してください。
周波数の確認は、独自のやり方でもいいし、一応pm_evaluation/pm_evaluation/cli/analyze_bag_frequencies.pyというros2 スクリプトがあるので、こちらを使っても構いません。

必要あれば、colcon build とsourceを行ってください。この際、少なくとも今回に限っては、よほど破滅的な変更が予測されない限りは、許可を求める必要はありません。

なお、ODriveは接続してありますが、モータ、エンコーダは繋いでいないので、これらの情報が取れなかった場合は無視してください。

Codex final response
====================

Jetson上でビルド、約67秒のbag記録、周波数・診断metadata検証まで完了しました。主要な開発成果は想定どおり記録されています。

記録先:

`/home/nvidia/patasmonkey_ws/bags/rosbag2_2026_09_21-18_07_57_codex_validation`

- Duration: 67.378秒
- Messages: 55,336
- Size: 約2.9 GiB（1 GB単位のMCAP 4ファイル）

主な実測周波数:

| トピック | 設定/想定 | 実測 |
|---|---:|---:|
| OAK stereo left/right | 20 Hz | 19.97 / 19.97 Hz |
| OAK color | 10 Hz | 10.16 Hz |
| OAK depth | 10 Hz | 10.01 Hz |
| OAK IMU | 125 Hz | 125.65 Hz |
| OAK frame診断 | 対応画像と同率 | 19.95～20.00 / 10.00 Hz |
| OAK IMU診断 | IMUと同率 | 125.75 Hz |
| OAK device info | 5秒周期 | 0.200 Hz |
| Wit IMU / mag | 100 Hz | 99.53 / 99.52 Hz |
| local EKF | 50 Hz | 49.76 Hz |
| global EKF | 10 Hz | 10.00 Hz |
| GNSS主要トピック | 1 Hz | 約1.00 Hz |
| OpenVINS track image | 20 Hz | 19.91 Hz |

OAK診断metadataの再生検証も `failures: []` で合格しました。

- MX ID: `19443010C10EF91200`
- USB: `UsbSpeed.SUPER`
- DepthAI: `2.30.0.0`
- IMU: `BNO086`
- sequence、device/host timestamp、露光、ISO、accel/gyro個別timestampを確認
- 左右画像のdevice timestamp差: median 14.5 µs、最大20 µs
- left/right/colorの累積sequence gap: 0
- Depthとgyroのgap表示は、20→10 HzのRGB-D変換とIMU rate変換による想定内の間引きで、診断レベルはすべてOK

解析成果物:

- `topic_frequency.csv`: 全トピックの周波数
- `oak_diagnostics_validation.json`: OAK診断metadata検証結果
- `openvins/`: console、timing、parameter dump、Git状態・差分、試行manifest

注意点として、車体がほぼ静止していたためOpenVINSは「加速度jerkなし」で初期化されず、`/ov_msckf/odomimu` と `/vio/odometry` は0件です。入力画像・IMU・track imageは正常なので、VIO出力周波数だけは今回の静止試験では検証対象外です。車体を安全に動かす次回試験で確認できます。

ODriveは接続・初期化され、`/motor_state` は約49.3 Hzでした。`/cmd_vel` は0件です。終了時にFoxyノードの一部がSIGINTを例外終了として表示しましたが、bagは正常に確定し、MCAP件数とmetadata件数は全トピックで一致しています。

実施したビルドはすべて成功しました。

- `depthai_driver`
- `pm_config`
- `pm_localization`
- `pm_evaluation`
- `pm_bringup`

試験後は `start-pm.service` がinactiveのままで、対象ROSプロセスの残留はありません。Jetsonのソースコード・設定・Git状態は変更していません。
