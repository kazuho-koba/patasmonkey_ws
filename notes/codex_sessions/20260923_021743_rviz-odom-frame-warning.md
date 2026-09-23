User prompt: rvizを起動すると以下のエラーがでます。
Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist
         at line 133 in /tmp/binarydeb/ros-foxy-tf2-0.13.14/src/buffer_core.cpp

Codex final response: これはRVizの異常ではなく、Fixed Frame=`odom`がまだTF treeに存在しない、という警告です。

今回の「最新localizationで再計算」構成では、terminal 1でlaunchした直後は正常に起こります。terminal 2のbag playerが`/clock`、`/tf_static`、wheel/VIO/IMUを流し始め、再計算localizerが`odom → base_link`をpublishした時点で解消するはずです。

まず3端末すべてで同じ`ROS_DOMAIN_ID=91`を設定し、terminal 1 → terminal 3（RViz）→ terminal 2（bag player）の順に起動してください。bag playerが再生を始めて数秒経っても警告が続く場合は、別端末で次を確認します。

```bash
export ROS_DOMAIN_ID=91
ros2 topic echo /clock --once
ros2 run tf2_ros tf2_echo odom base_link
```

期待値は、`/clock`が一度出力され、`tf2_echo`が時刻付きのtransformを連続表示することです。

- `/clock`が出ない: bag playerが起動していない、終了した、またはDomain IDが異なります。
- `/clock`は出るが`tf2_echo`が待ち続ける: `/tf_static`、`/wheel/odometry`、`/vio/odometry`、`/wit/imu`のいずれかをplayerへ渡していない可能性があります。`/tf`は渡さないでください（記録時の古い`odom → base_link`と競合します）。
- bag再生の完了後に出る: replay localizerが停止してTFが無くなるため正常です。bagを最初から再生する間だけ確認してください。

なお、旧odomをそのまま使う確認用の`terrain_mapping_old_bag.launch.py`でも、bag再生前は同じ警告が出ます。この場合は`ros2 bag play`へ`/tf`と`/tf_static`を含めて再生開始すれば解消します。
