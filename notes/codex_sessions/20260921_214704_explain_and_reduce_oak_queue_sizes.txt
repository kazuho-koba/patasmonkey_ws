User prompt
===========

queueサイズを増やすのはいいですが増加幅が大きく変更が極端ではないですか？設定値にはなにか根拠があるのでしょうか？

Codex final response
====================

ご指摘のとおりです。最初の`120`は恒久設定としては大きすぎました。これは「今回観測した最大4.6秒の欠落を20 Hzで全保持する」ための上限値で、`4.6 × 20 = 92`フレームに余裕を加えた値です。異常停止をbufferで隠す値であり、通常運用の根拠としては適切ではありません。

特にcolor/depthを120にしても、現行ノードはbacklogのうち最新フレームだけをRGB-Dとしてpublishするため、メモリを増やすだけで有効ではありません。最大でstereo約59 MiB、color/depth約147 MiB、合計約200 MiB超のhost RAMを使い得る点も過剰です。また、4.6秒分の画像をcallback復帰後に一気にpublishすると、DDS/OpenVINS側のqueueをburstで圧迫する懸念があります。

したがって、開発PC側の未コミット修正を保守的な値へ変更しました。[oakd_vio_rgbd_node.py](/home/kazuho/ros2_ws/src/depthai_driver/depthai_driver/oakd_vio_rgbd_node.py)

| queue | 変更前 | 見直し後 | 根拠 |
|---|---:|---:|---|
| stereo left/right | 2 | 20 | 20 Hzで1秒。元の100 msから短いhostジッタへ耐性を持たせる。 |
| color/depth | 1 | 10 | 10 Hzで1秒。RGB-Dの意図したpublish周期に対応。 |
| IMU | 50 | 250 | 125 Hzで2秒。IMU batchの短い遅延を吸収。 |

主要な対策はqueueを巨大化することではなく、`tryGet()`を`tryGetAll()`へ変更して、timer callbackが一時的に遅れた後もqueue内の全データをdrainすることです。stereoはDepthAI sequence番号で左右を一致させてからpublishし、IMUも全batchをdrainします。topic名、message型、frame ID、timestamp仕様は変えていません。

4.6秒の停止は正常なジッタではありません。20/10/250は短時間の遅延を吸収しつつ、これを超える異常停止は従来どおりdiagnostic sequence gapとして観測できる、という意図です。再計測でgapが残るなら、次はbufferをさらに増やすのではなく、`oakd_vio_rgbd_node`のpoll処理時間・queue回収件数をdiagnosticへ追加し、bagあり/なし比較でcallback停止原因を特定します。

開発PCコンテナで構文確認と`colcon build --packages-select depthai_driver`は成功しました。変更は`/home/kazuho/ros2_ws/src/depthai_driver`に未コミットのままです。
