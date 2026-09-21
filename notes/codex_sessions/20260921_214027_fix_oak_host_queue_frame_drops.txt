User prompt
===========

track_frequencyを20から21にしたら、oak-dのモノクロ画像の周波数がほぼ20Hzだったのから19Hzに落ちたように見えます。これが例の4秒分ドロップによるものでしょうか？マシンパワーの限界でないのであればこれの原因究明と解決をお願いします。

Codex final response
====================

はい。19 Hzは`track_frequency: 21.0`がOAK-Dの撮像周波数を下げたのではなく、今回記録で発生したOAKフレーム欠落を含む平均値です。

OAK left画像の有効区間は199.347秒、記録数は3,805枚でした。20 Hzなら約3,988枚になるはずで、その差183枚は、`/oak/diagnostics/left_frame`の`cumulative_sequence_gap=183`と完全に一致します。したがって、約19.08 Hzという平均値は4.6秒を最大とする累計183フレームのdropによるものです。

`track_frequency`はOpenVINSが受け取った画像を何Hzまで処理するかの上限であり、OAK-Dの`MonoCamera.setFps(20)`やUSB撮像設定を変更しません。21 Hz化後のOpenVINS画像更新は18.79 Hzで、ドライバから届いたstereo入力19.08 Hzの約98.5%を処理できています。従来の13.5 Hzへの不要なOpenVINS間引きは解消済みです。

マシン全体の性能限界も主因ではありませんでした。記録区間のJetsonはCPU平均53.2%（8コア平均）、idle最低21%、I/O wait最大1%、CPU/GPU最高57.0/57.5℃、RAM最大3.06/14.89 GiBでした。OpenVINS画像処理はp95 20.48 ms、p99 27.99 msで、50 ms超は2回だけです。

原因は`depthai_driver`のhost-side queue処理です。従来実装は、2 ms周期の単一Python callbackで画像変換・stereo/RGB-D/IMU/診断publishをすべて行う一方、left/right queueを`maxSize=2, blocking=False`とし、各callbackで`tryGet()`を1件だけ読んでいました。callbackが100 msを超えて遅れた時点で古いstereoフレームが上書きされます。今回の最大4.6秒停止に対して2フレーム（100 ms）のbufferでは不十分です。OAK diagnosticのsequenceとdevice timestampの同時ジャンプは、このdriver経路でpublishできなかったことを示します。

開発PCの外部driver workspaceで、[oakd_vio_rgbd_node.py](/home/kazuho/ros2_ws/src/depthai_driver/depthai_driver/oakd_vio_rgbd_node.py)を修正しました。

- left/right/color/depthのDepthAI host queueを`maxSize=120`へ拡大（20 Hzで約6秒のstereo保護）。
- IMU queueを`50`から`500`へ拡大。
- `tryGet()`を`tryGetAll()`へ変更し、callback復帰時に蓄積分を全回収。
- left/rightをDepthAI sequence numberで一致させてからpublishし、片側だけを組み合わせないように変更。
- IMU batchもすべてdrain。
- topic名、message型、frame ID、timestamp仕様、OpenVINS向けinterfaceは変更していません。

この修正は根本的に「callback遅延をなくす」のではなく、短時間のhost遅延が発生してもVIO入力を捨てないようにします。bufferを超える6秒以上の連続停止があればdropはなお起こり得ますが、その場合もdiagnostic sequenceで確認できます。

検証結果:

- 開発PCコンテナのDepthAI 2.30.0.0で`DataOutputQueue.tryGetAll()`が利用可能であることを確認。
- `depthai_driver`の`colcon build --packages-select depthai_driver`は成功。
- Python構文確認と`git diff --check`は成功。
- `colcon test`は失敗しましたが、既存の`tools/test_depth_visualize.py`がpytestに誤収集され、コンテナ内でOAKデバイスを開こうとして失敗する既存テスト構成が原因です。今回修正箇所のtest failureではありません。

現在、変更は開発PCの`/home/kazuho/ros2_ws/src/depthai_driver`にだけあり、未コミットです。Jetsonへのデプロイはまだ行っていません。

Jetsonへこのdriver変更をcommit/push/pullして`depthai_driver`を再ビルドした後、同じ全topic bag記録を再実行してください。合格基準は、`/oak/diagnostics/left_frame`の`cumulative_sequence_gap`が0、または少なくとも183から大幅に低下し、left/rightの実測が20 Hz近傍、OpenVINS pose/path/pointsが入力画像に追従することです。
