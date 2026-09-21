User prompt
===========

>画像更新系のpose/path/point cloudは約13.5 Hz
といいますが、これは本来何Hzであるべきものですか？入力画像と同じ周波数（今回でいうと20Hz）であるべきものでしょうか？
そうであれば、原因の分析を行ってください。必要ならスクリプトに記録データの追加、データ再記録等も行ってください。

Codex final response
====================

はい。この構成では、`poseimu / pathimu / points_*` は本来ほぼ20 Hzであるべきです。原因も特定できました。

OpenVINSでは、stereo画像を1組処理するたびに `visualize()` が呼ばれ、その中でpose、path、point cloudを各1回publishします。[ROS2Visualizer.cpp](/home/kazuho/ros2_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.cpp:472) [ROS2Visualizer.cpp](/home/kazuho/ros2_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.cpp:246)

現在の `track_frequency` は20 Hzです。[estimator_config1.yaml](/home/kazuho/Projects/patasmonkey_ws/src/pm_config/config/oak_d_s2/estimator_config1.yaml:87)

原因はOpenVINSのフレーム間引き判定です。

```cpp
time_delta = 1.0 / track_frequency;  // 0.05秒
if (timestamp < last_timestamp + time_delta) {
    return;
}
```

この比較には時刻揺らぎの許容差がありません。[ROS2Visualizer.cpp](/home/kazuho/ros2_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.cpp:540)

今回のbagを再生し、画像header timestampを直接解析した結果は次のとおりです。

- 左画像: 3,625枚
- フレーム間隔中央値: `49.999952 ms`
- 最小値: `49.947977 ms`
- 50 ms未満の区間: 2,219 / 3,624

入力は実質20 Hzですが、中央値が理論値50 msより約48 ns短いため、OpenVINSは多数の正常フレームを「20 Hzを超えた」と判定して破棄しています。

同じ判定をbagの画像timestampへ適用すると、結果は以下でした。

- 許容差なし: 3,625枚中2,451枚を受理（約13.2 Hz）
- 初期化後区間: 予測2,133枚、実際のpose出力2,135件
- 1 µs許容: 3,493枚を受理
- 10 µs許容: 3,610枚を受理
- 100 µs許容: 3,625枚すべてを受理

予測と実測がほぼ完全に一致するため、主原因はCPU性能、stereo同期、rosbagの書き込み負荷ではなく、この境界値比較です。OpenVINS処理時間も平均17.7 ms、p95 25.9 msで、通常は50 ms周期内に収まっています。

なお、`/ov_msckf/odomimu` が約122 Hzなのは正常です。これは画像更新ではなく、IMU callbackごとの高速propagation結果です。[ROS2Visualizer.cpp](/home/kazuho/ros2_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.cpp:440)

修正方法は2通りあります。

1. 設定だけの暫定対応

   `track_frequency: 21.0` にします。判定周期が約47.62 msになるため、実測最小49.948 msの20 Hz画像をすべて通せます。再ビルドは不要ですが、`pm_config` の再インストールは必要です。

2. OpenVINSコードの恒久対応

   比較へ100 µs程度の許容差を追加し、受理・間引きフレーム数も診断出力します。例えば概念的には次の判定です。

```cpp
if (timestamp + timestamp_tolerance < last_timestamp + time_delta) {
    return;
}
```

固定100 µsでも今回のデータには十分ですが、パラメータ化した方が再利用性があります。monocular側にも同じ判定があるため、stereo側と共通化するのが適切です。

既存bagだけで原因と再現条件を確定できたため、新たなセンサーデータの記録や本番コード変更は行っていません。次の作業としては、まず低リスクな `track_frequency: 21.0` で再検証し、その後に許容差とdropカウンターをOpenVINSへ実装する順序を推奨します。
