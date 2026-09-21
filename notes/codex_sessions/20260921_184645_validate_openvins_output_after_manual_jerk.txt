User prompt
===========

再検証をしたいです。次はセンサが起動したら、僕が手動でjerkを加えるので、OpenVINSの出力も検証対象にしてください。
準備ができたら、jerkを加えるよう僕に求めてもらえますか？

User prompt
===========

jerkを加えました

Codex final response
====================

再検証が完了しました。手動jerkによってOpenVINSは正常に初期化され、推定結果もbagへ保存されました。

記録先:

`/home/nvidia/patasmonkey_ws/bags/rosbag2_2026_09_21-18_41_03_openvins_jerk_validation`

- 記録時間: 195.941秒
- メッセージ数: 276,007
- サイズ: 約8.0 GiB
- MCAP: 9ファイル

OpenVINSログでは次を確認しました。

- `USING STATIC INITIALIZER METHOD!`
- `successful initialization in 0.0044 seconds`
- 初期位置: `(0, 0, 0)`
- 初期化後のstate/timing記録: 各2,137行
- `/ov_msckf/odomimu` の実メッセージ受信: 成功

主なOpenVINS周波数:

| トピック | 件数 | active区間の実測 |
|---|---:|---:|
| `/ov_msckf/odomimu` | 19,345 | 122.31 Hz |
| `/vio/odometry` | 19,371 | 122.66 Hz |
| `/ov_msckf/poseimu` | 2,135 | 13.49 Hz |
| `/ov_msckf/pathimu` | 2,135 | 13.49 Hz |
| `/ov_msckf/points_msckf` | 2,135 | 13.49 Hz |
| `/ov_msckf/points_slam` | 2,135 | 13.49 Hz |
| `/ov_msckf/trackhist` | 2,653 | 14.24 Hz |

OAK入力は以下でした。

- stereo left/right: 約19.46 Hz
- OAK IMU: 約123.18 Hz

したがって、IMU駆動のOpenVINS odometryは入力IMUとほぼ同じ周波数で正常です。一方、画像更新系のpose/path/point cloudは約13.5 Hzで、stereo入力約19.5 Hzより低くなっています。OpenVINSは動作していますが、入力画像の全フレームを推定更新には使えていません。

OpenVINSの処理時間は以下でした。

- total平均: 17.72 ms
- median: 17.31 ms
- p95: 25.88 ms
- p99: 35.15 ms
- 最大: 101.32 ms

通常は20 Hz周期の50 ms以内なので、推定計算そのものが恒常的に遅いわけではありません。ただしstate更新間隔が100 msを超えた箇所が310回あります。ROS callback、stereo同期、増大するPath/PointCloudのpublish、8 GiB規模の同時bag書き込みなどを次に切り分ける価値があります。

解析結果はbag内の `topic_frequency.csv`、詳細なOpenVINS処理時間は `openvins/timing.txt`、初期化ログは `openvins/console.log` に保存されています。

終了時にSSH端末からのCtrl-Cがリモートlaunchへ伝わらず、一度launchが残りました。その後、特定したlaunch PIDへSIGINTを送り、孤児化したOpenVINS PIDも個別に停止しました。現在は対象プロセスの残留なし、`start-pm.service` もinactiveです。bagのmetadataは正常に確定し、Git管理ファイルは変更していません。
