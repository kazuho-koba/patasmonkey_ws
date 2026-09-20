# VIO破綻のbag比較と次回計測

`tools/diagnose_vio_bags.py` はMCAPの画像・IMU・VIO・wheel・GNSS速度・ログを抽出する。
既存 `plot_bag_trajectories.find_mcap_files` を再利用する。Foxyコンテナで実行する。
本番ドライバ、OpenVINS、本番launch/configは変更しない。

## 実行

```bash
docker exec --user 1000:1000 --env HOME=/tmp patasmonkey_foxy_dev bash -lc '
  source /opt/ros/foxy/setup.bash
  source /workspaces/ros2_ws/install/setup.bash
  source /workspaces/patasmonkey_ws/install/setup.bash
  cd /workspaces/patasmonkey_ws
  python3 src/pm_evaluation/tools/diagnose_vio_bags.py \
    --bag bags/rosbag2_2026_07_26-09_17_38 \
    --bag bags/rosbag2_2026_08_10-18_42_30 \
    --output src/pm_evaluation/results/vio_failure_comparison
'
```

- `--bag` は繰り返し指定できる。
- `--sample-hz 2`: 画像品質・簡易LK指標の評価頻度。タイムスタンプは全件評価。
- `--plot-only`: 抽出済みCSVから図と集計だけ再生成。
- `--startup-only`: 抽出済みCSVと冒頭45秒の画像から初期化拡大図を生成。
- `--replay-startup`: 現行OpenVINSバイナリ・configで冒頭区間を再生し、内部状態・DEBUGコンソールを記録。
- `--replay-duration 45`, `--rate 0.5`: 冒頭再生の終了時刻（bag開始から）と再生速度。
- `--freeze-calibration`: 冒頭再生のカメラ内部/外部/時差、IMU内部/g感度のオンライン校正を全てfalseにする。
- `--config`: 冒頭再生に使う設定。変更はROSパラメータ上書きで行い、設定ファイルそのものは編集しない。
- `--replay-tag`: 再生結果ディレクトリ名を明示し、冒頭/全区間などの条件を同じ処理で保存する。
- `--plot-offline-overlay`: オフラインVIO、GNSS、wheel vx + Wit wz EKFを重畳する。
- `--comparison-results`: GNSS・wheel+IMU解析結果ディレクトリ。`--overlay-replay-tag`で重畳する再生結果を選ぶ。
- `--fit-start-s`, `--fit-end-s`: VIOをGNSSへ合わせる区間。平行移動+yawだけを推定し、scaleは変更しない。

冒頭再生はROS_DOMAIN_ID=91 / localhostのみで実行する。同じドメインで他の再生を同時実行しない。
生成先 `replay_current_config` / `replay_frozen_calibration` の上書きを拒否する。
当時の設定・バイナリ・Jetson負荷・購読欠落を再現するものではなく、入力に対する対照実験である。

## 指標の限界

- 時刻0はbag記録開始であり、launch開始やOpenVINS内部の初期化完了時刻ではない。
- `/poseimu` の初回記録は最初に観測できた画像更新側出力。正確な初期化時刻は内部ログが必要。
- 加速度ノルムは重力を含む。車体の並進加速度と同一ではない。
- Laplacian分散はテクスチャ量にも依存する。値が低いだけでモーションブラーと断定しない。
- 特徴点候補はOpenCV goodFeaturesToTrack (max 300)、5×5区画の占有率と、約0.5秒間隔の前後整合LKを使用。
  OpenVINSのHISTOGRAM処理、FAST/KLT、ステレオ拘束、外れ値検定、採択点数とは異なる。
- rosbagで受信したことはOpenVINSも同じメッセージを受信した証拠にはならない。
- OAK左右画像はドライバが同一headerを付与するため、header一致だけでは撮像同期を証明できない。

## 次回計測で優先して残すもの

| 優先度 | 記録対象 | 目的・取得方法 |
|---|---|---|
| 最優先 | 当日のestimator/imu/imucam YAMLの実ファイル、実行コマンド、全パラメータdump、git SHA、実行バイナリの版 | parameter_eventsにはconfigパスは残るが、OpenCV YAMLから直接読む全内容は残らない。カメラS/Nと使用校正ファイルの一致も記録 |
| 最優先 | OpenVINS標準出力・標準エラー、verbosity=DEBUGの初期化区間 | PRINT_DEBUG/INFOは独自stdoutロガーで、/rosoutだけでは保存できない。初期化のIMU励起、disparity、bg/ba、採択点・更新処理を確認 |
| 最優先 | `save_total_state:=true` と `filepath_est` / `filepath_std` | q,p,v,bg,ba,cam-IMU時差、内部/外部校正、分散の時系列をファイル保存。launch既定falseなのでYAMLだけでなく実行時上書きを明示。実験ごとの別ディレクトリを指定（既存ファイルは初期化時に削除される） |
| 高 | `/ov_msckf/trackhist` | 実際に追跡した点の画像。現行launchではコメントアウト。人物への追跡集中、点の寿命、遮蔽を目視確認 |
| 高 | `/ov_msckf/points_msckf`, `/ov_msckf/points_slam` | 使用/保持された3D特徴の状況。後者はmax_slam=0なら空。全候補のID/棄却理由を表すtopicではない |
| 高 | カメラ左右それぞれのdevice timestamp、sequence number、exposure、gain、カメラS/N | 現行DepthAIドライバに診断出力を追加する必要あり。左右の実撮像同期と露光変化を検証 |
| 高 | 加速度・角速度それぞれのdevice timestampと生値、ROS送信時刻 | 現行ドライバはaccel時刻をcombined IMUに代表使用。gyroとの時差を後から復元できないため診断topic/sidecar追加 |
| 高 | 各画像更新のtracked/stereo対応/三角測量成功/MSCKF採択/chi-square棄却点数、残差、正規化innovation、最終画像更新時刻、初期化状態 | このローカルROS2実装には一式を出す専用topicがない。OpenVINS内部の診断publisher/CSV追加が必要。trackhistだけで代替しない |
| 中 | `record_timing_information:=true`, `record_timing_filepath` | OpenVINS処理時間CSV。購読キューの長さ・取り落とし数は別途計測 |
| 中 | Jetson tegrastats、CPU/GPU/メモリ/温度、USB接続・切断ログ、キューdrop数 | 負荷や輸送経路による欠落の切り分け。/diagnosticsだけでは全項目を含まない |
| 中 | `/oak/.../camera_info` 相当の実際に使用したintrinsics、歪み、解像度、IMU種類 | 現行customドライバではCameraInfo publisherなし。EEPROM calibrationの保存またはpublisher追加 |

今回すでに存在する左右mono/RGB/Depth、OAK IMU、Wit IMU、motor_state、wheel、raw OpenVINS pose/odom、GNSS速度、TF、/rosout、/parameter_eventsも継続保存する。
depthはOpenVINSステレオ入力に使われていないが、人物/近接物体の判定に役立つ。

## 次回の対照計測

1. OAK露光が安定してから、静止した車体・人物の入らない視野で初期化を待つ。初期化確認は出力開始だけでなく静止時速度・姿勢安定性も確認。
2. 同じ条件で人が前を横切る場合を比較し、動的物体の影響を確認する。
3. 同一bagで現行校正ONと、検証済み校正値を固定した条件を比較する。平面走行だけでは多数の校正パラメータの観測が弱い可能性がある。
4. 振動区間と静止区間を別に記録。装置を手で触れた時刻、車輪を駆動した時刻をevent topicなどで残す。

参考：
- https://docs.openvins.com/gs-calibration.html
- https://docs.openvins.com/classov__init_1_1InertialInitializer.html
- https://docs.openvins.com/eval-timing.html
- ローカル `open_vins/ov_msckf/src/ros/ROS2Visualizer.cpp` のpublisherとstate保存実装
- ローカル `open_vins/ov_core/src/utils/print.h` のstdoutロガー
