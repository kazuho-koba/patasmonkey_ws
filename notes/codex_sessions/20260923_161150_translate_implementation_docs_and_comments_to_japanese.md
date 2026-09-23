User prompt
AGENTS.mdにも書きましたが、READMEとコメントについては日本語でお願いしたいです。ここまでの実装成果に入っているコメントや説明書きは日本語に変更してもらえますか？

Codex final response
はい。今回の自己位置推定、OAK-D診断、OpenVINS検証で追加・変更したREADME、docstring、処理コメント、設定YAMLコメントを日本語へ統一しました。topic名・frame名・パラメータ名・message型・コード識別子は、ROS/OpenVINSとの対応を保つため英語のままです。

主な対象は以下です。

- `pm_localization`のVIO速度／高さゲート、姿勢・高さ観測器、local odometry composer、heading初期化器、GNSS品質ゲート。
- `pm_config`の水平EKF、GNSS拘束global EKF、heading初期化、GNSS gate、navsat設定。
- `depthai_driver`のOAK-D診断nodeと[README_diagnostics.md](/home/kazuho/ros2_ws/src/depthai_driver/README_diagnostics.md)。
- `open_vins`のOAK-D S2設定にある`track_frequency`説明と、ROS2 frontendのmonocular/stereo間引き処理のコメント。

READMEとして前回追加したPatasmonkey側のlocalization・bringup・再生評価、外部workspace側のPatasmonkey統合READMEも日本語です。

処理説明も日本語化済みです。例えば、VIO gateの検査順・隔離／再採用、z保持出力の仕組み、heading seedからdatum設定までの一回限りの流れ、GNSS候補の検査・warm-up・共分散置換、OAK queue drainとmetadata対応付けをコード上で追えるようにしています。

`git diff --check`はPatasmonkey本体、`depthai_driver`、`open_vins`で成功しました。更新したPython node群の`py_compile`、OpenCV FileStorageによるOpenVINS YAML読み込みも成功しています。
