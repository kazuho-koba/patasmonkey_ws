【ユーザープロンプト】
ありがとうございます、では、B側（frequencyを21としたケース）で、改めてlaunchファイル起動、設定されている全データをbag記録の上、周波数を確認してマシンパワーの限界でスタックしていないかなどをチェックしてもらえますか？

【Codex最終回答】
完了しました。21 Hz設定の実機標準launchで210秒間・9.3 GiBのbagを記録し、全設定トピックの周波数、OpenVINS処理時間、OAK診断、Jetson負荷を確認しました。

記録先（Jetson）:

`/home/nvidia/patasmonkey_ws/bags/rosbag2_2026_09_21-19_31_30_track21_live`

主な成果物:

- `topic_frequency.csv` — 全トピックの実測周波数
- `openvins/timing.txt`、`state_estimate.txt`、`console.log`
- `openvins/ov_msckf__run_subscribe_msckf.yaml` — 実効パラメータ
- `trial_manifest.json`、config/git snapshot

Jetsonの`pm_config`は`track_frequency: 21.0`へ更新し、`pm_config`だけ再ビルド済みです。変更はJetsonローカルリポジトリに未コミットで、開発PC側のリポジトリには反映していません。

| 系統 | 実測周波数 |
|---|---:|
| OAK stereo left/right | 19.08 Hz / 19.08 Hz |
| OAK IMU | 121.26 Hz |
| OAK frame diagnostics left/right | 19.06 Hz / 19.05 Hz |
| OAK color/depth | 9.55 Hz / 9.53 Hz |
| OpenVINS pose/path/points | 18.79 Hz |
| OpenVINS IMU odometry | 118.97 Hz |
| VIO adapter odometry | 119.44 Hz |
| Wit IMU / magnetometer | 99.59 Hz / 99.54 Hz |
| wheel odometry / local EKF | 49.64 Hz / 49.99 Hz |
| global EKF | 10.00 Hz |
| GNSS fix/navheading/navpvt | 1.00 Hz |

OpenVINSは手動jerk後に正常初期化しました。画像更新は18.79 Hzで、従来の13.5 Hzの間引きは解消されています。入力stereo 19.08 Hzに対して約98.5%です。

OpenVINSの画像処理時間は平均14.37 ms、p95 20.48 ms、p99 27.99 ms、最大63.66 msでした。50 msを超えた更新は2件だけで、queue full、tracking lost、NaN、異常終了のログはありません。OpenVINS処理が周期的に詰まっている証拠はありません。

Jetsonの記録区間211サンプル（1 Hz）の負荷は、CPU平均53.2%（8コア平均）、最大83.4%、CPU idle最低21%、I/O wait最大1%、RAM最大3.06/14.89 GiB、CPU/GPU最高温度57.0/57.5℃でした。CPU周波数は1.19〜2.265 GHzで推移しています。熱、メモリ、ディスクI/O、CPU全体の飽和によるスタックは確認されませんでした。

ただし、OAKの入力には無視できない欠落があります。左フレーム診断の`sequence_num`は210秒で累計183フレーム飛んでおり、最大4.6秒（91フレーム）の連続欠落が1回ありました。他にも1.35秒、1.20秒、1.60秒の欠落があります。Image header timestampとDepthAI device timestampにも同じギャップがあるため、rosbagの書込み遅延だけではありません。

この診断は「DepthAI sequenceのフレームがドライバからpublishされなかった」ことを証明しますが、OAKセンサ内部、USB/XLink、DepthAI host queue、Pythonドライバの遅延のどれかまでは単独では断定しません。今回の全体CPU余力とログからは、`oakd_vio_rgbd_node`の実装が有力です。このノードは2 ms周期の単一Python callbackで画像変換・RGB-D・diagnostics publishを行い、left/right queueを`maxSize=2, blocking=False`で読むため、一時的にcallbackが遅れた場合に古いフレームを捨てます。

したがって今回の結論は次の通りです。

- `track_frequency: 21.0`は有効で、OpenVINS側の不要な13.5 Hz間引きは解消された。
- マシン全体は性能限界でスタックしていない。
- 残る入力欠落はOAK/DepthAIドライバ経路で起きており、VIOの破綻率・精度を改善する次の優先調査対象である。

記録対象に含まれていて0件だったのは`/imu`、`/gps/filtered`、`/odometry/gps`、`/cmd_vel`、`/cmd_vel_joy`、`/emergency_stop`、`/navsvin`、`/set_pose`です。これはこの試行でpublishがなかったためで、rosbagの取りこぼしではありません。metadataの全トピック件数とMCAP実読件数は一致しました。

記録終了後、今回起動したROSプロセスはすべて停止しました。バックグラウンド起動したlaunchがSIGINTを無視して子ノードを孤児化したため、対象PIDだけへSIGTERMを送り停止しています。

次の調査としては、設定を変更せずにまず「bagなし」と「全bag記録あり」のOAK sequence gapを比較し、記録負荷との因果を確認するのがよいです。その後、必要ならDepthAI driverを`tryGetAll()`でqueueを毎周期drainする、stereo queue容量を増やす、画像publishとRGB-D/diagnosticsを分離する、callback実行時間とqueue滞留数を診断topicへ追加する順で対処できます。
