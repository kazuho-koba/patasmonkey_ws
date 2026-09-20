# 7月正常例と8月VIO破綻例の比較

対象: `rosbag2_2026_07_26-09_17_38` / `rosbag2_2026_08_10-18_42_30`。
時刻はすべて各bagの記録開始からの秒。launch実行開始時刻とは異なる。

## 結論と確度

**8月は走行中の振動で初めて破綻したのではなく、初期化直後・車輪駆動開始前にOpenVINS本体が発散していた。**
raw `/ov_msckf/odomimu` とアダプタ後 `/vio/odometry` の両方に同程度の発散がある。
heading補正、local EKF、wheel速度外れ値はこの最初の発散原因ではない。

**有力な引き金は、露光変化・動く近接人物を含む画像で早期初期化したこと、および弱い運動励起での内部校正/バイアス誤推定。**
ただし当時の内部状態・採択特徴・実設定ファイルがないため、原因を一意に断定できない。
現行環境で冒頭45秒だけを再生した場合は巨大発散を再現しなかったが、全区間再生では現行設定・校正固定の双方が走行中に発散した。

## 起動・破綻タイムライン

| 観測 | 7月 | 8月 |
|---|---:|---:|
| OpenVINS config_pathのparameter event | 4.43 s | 4.81 s |
| OAK左右画像初回header | 11.67 s | 12.58 s |
| OAK IMU初回header | 11.75 s | 12.56 s |
| 最初に記録されたOpenVINS poseimu | 36.47 s | 19.28 s |
| 最初に記録されたOpenVINS odomimu | 36.52 s | 19.33 s |
| 画像開始→最初のposeimu | 24.80 s | 6.70 s |
| ODrive接続完了ログ | 10.47 s | 36.55 s |
| wheel速度初回0.2 m/s超 | 35.63 s | 48.11 s |
| raw VIO速度初回2 m/s超 | 57.70 s（wheelも約2 m/s） | 22.06 s（駆動開始前） |
| raw VIO速度初回10 m/s超 | なし | 26.00 s |

初回出力時刻は内部初期化時刻そのものではない。OpenVINSは初期化履歴時刻、画像更新、fast propagationの都合で出力タイミングが異なる。

8月はODrive接続失敗・再接続を繰り返しており、36.55秒まで接続完了していない。
画像・両IMUも23～30秒の車体静止を支持する。GNSS速度は低速時にも雑音があるため、単一速度閾値だけで静止を断定していない。

## IMUと推定姿勢の矛盾

8月23～30秒:

- OAKおよびWitの記録gyro 3成分は全て0。
- OAK加速度平均は `[-9.7545, +0.1728, -0.4894] m/s²`、3軸標準偏差ノルム0.0204 m/s²。
- Wit加速度の3軸標準偏差ノルム0.00573 m/s²。
- それでもOpenVINS quaternionの回転角は約15.48°変化。
- raw OpenVINS出力の角速度ノルムは中央値0.03860 rad/s（約2.21°/s）。

現行 `Propagator.cpp` の出力角速度は
`R_GYROtoIMU * Dw * (gyro - bg - Tg * corrected_accel)`。
記録gyroが0なのに補正後角速度が大きいのは、推定バイアス/内部校正補正が誤った旋回を生成している説明と整合する。
当時のbg/Dw/Tgがないので、bgだけに責任を限定できない。

初回出力から最初の数秒は水平位置・姿勢が崩れ、その後z方向も大きく発散する。
単純な初期heading差や地図への表示回転では、この位置・速度の巨大な増加は説明できない。

7月の37～45秒では姿勢変化約0.83°、補正後gyroノルム中央値0.00037 rad/sで、8月と明確に異なる。

## 画像で確認できた差

`startup_frames.png` に約2秒刻み、通常contact sheetに約15秒刻みの画像を保存した。

- 両bagとも画像開始直後に白飛び・自動露光の過渡がある。白飛びの存在だけでは8月特有の原因にならない。
- 7月は静止背景（近くの壁・舗装・建物）が豊富で、露光が落ち着いた後、約36秒に初回VIO出力。
- 8月は広い舗装と空が多く、近接した人物が画面内にいる。18.58秒には別の人物の脚が視野手前を大きく遮蔽し、19.28秒の初回VIO出力に近接する。
- 8月18～20秒のoffline LK生存率は約0.64～0.90と変動する。一方22～30秒にはほぼ1に戻るのにVIOは発散を続ける。
  「ずっと画像が追跡不能だった」という説明だけでは不十分。
- 全区間の左画像Laplacian分散中央値は7月2889.85、8月1005.73。特徴候補の5×5区画占有率中央値は7月0.96、8月0.76。
  空・平坦路面・遠方物体が多い8月の方が静的な拘束が弱い可能性がある。
- LK生存率の全区間中央値は7月0.773、8月0.787。8月の方が全体に追跡不良という結果ではない。

これらは画像から再計算した代理指標で、OpenVINSの実際のFAST/KLT追跡数・ステレオ採択数ではない。
Laplacian分散の低さはテクスチャの差を含むため、ブラーの確定指標として使っていない。
RGB画像も同じ場面・人物を確認する補助として抽出した。OpenVINSの直接入力は左右monoとOAK IMU。

## 加速度・欠落・同期

- 8月の大きな衝撃は約88.60秒、OAK加速度ノルム最大45.75 m/s²（重力含む）。破綻開始から約66秒後であり、最初の原因ではない。
- 初期化付近18～20秒のOAK加速度ノルム最大10.35 m/s²、gyroノルム最大0.104 rad/s。小さな揺れは存在する。
- 7月の初期化付近34～37秒は加速度ノルム最大10.95 m/s²、gyroノルム最大0.297 rad/s。
- 冒頭30秒の最大画像header gapは両bag約50 ms。最大IMU gapは7月47.6 ms、8月55.5 ms。
  8月の初期発散にだけ対応する秒単位の入力停止は見つからない。
- 全区間最大OAK IMU gapは8月444 msだが約250.73秒で、最初の発散より後。
- 画像のrecord-header中央値は7月-33.2 ms、8月-58.8 ms、OAK IMUは-40.3 ms / -65.3 ms。
  負値は通常の輸送遅延とは説明できず、device→ROS時計対応の偏りを示す。画像とIMUに共通する部分もあるので、それだけで画像-IMU相対時差を断定しない。
- custom DepthAIドライバは最初のdevice timestampをROS現在時刻へ合わせ、左右に左画像の同一stampを付ける。
  Combined IMUはaccel timestampを採用する。元の左右sequence/gyro timestampがないため、真の同期誤差は未確定。

186.73秒のwheel異常はODrive再接続直後の別問題。VIOは約22秒から既に破綻している。

## 現行OpenVINSでの対照再生

8月bag開始から45秒まで、画像・OAK IMUだけを隔離domain=91へ記録受信順で再生。
再生rate=0.5、現行config/バイナリ、DEBUGコンソール・全状態・timingを保存。
二条件は現行設定と、5つのオンライン校正フラグをfalseにした設定。実ファイルは書き換えていない。

| 条件 | 19.5～44.5秒の最大速度 | 44.5秒時点の初回位置からの変位 |
|---|---:|---:|
| 当時のraw OpenVINS記録 | 122.26 m/s | 1060.20 m |
| 現行設定で再生 | 0.492 m/s | 1.126 m |
| オンライン校正固定で再生 | 0.289 m/s | 0.350 m |

現行設定再生の最終cam-IMU推定時差は7.38 ms。
校正固定で静止時ドリフトが減る結果だが、一回ずつの比較であり本番修正の効果を保証しない。
**冒頭45秒では当時の巨大発散を再現できなかったので、校正ONが根本原因と断定しない。**
現行PCの処理能力、再生速度、thread scheduling、実際に購読したフレーム、当時YAML・バイナリの違いが残る。
初期化時刻の近さや画像内容と矛盾しない一方、人物通過だけで常に発散するとも言えない。

その後、同じ2条件を8月bag全区間へ延長したところ、双方とも走行中に発散した。
校正固定条件の速度中央値/最大値は11.90/72.93 m/s、位置のx/yレンジは3444.68/1905.62 m。
現行設定条件は36.70/199.48 m/s、x/yレンジ12676.2/6990.6 mで、校正固定は悪化を遅らせるものの実用的なVIOにはならなかった。
両条件ともbag開始約250.73秒のOAK IMU 444 ms欠落付近でOpenVINSの伝播時刻assertにより終了した。

したがって「破綻しなかったオフラインVIO」は、正確にはbag開始63.5秒までの初期区間だけを指す。
48.0～63.5秒をGNSSへ平行移動+yawのみ（scale補正なし）で合わせたRMSEは0.51 mだが、全出力区間では1471.58 mである。

## 不足情報と次回計測

両bagのparameter_eventsには同じ`estimator_config1.yaml`パス、verbosity=WARNING、save_total_state=falseがある。
パスの一致はファイル内容の一致を証明しない。現行設定では全5系統のオンライン校正がtrue、init_dyn_use=false、init_window_time=3秒、num_pts=60、max_clones=5、max_msckf_in_update=10、try_zupt=false。
これらは現在のファイルから分かる設定で、当時の完全な設定をbagから復元した値ではない。

実特徴topic / trackhist / CameraInfoは両bagに記録されていない。
OpenVINS名の/rosoutログも0件。独自PRINTロガーはstdoutへ出すため、/rosoutだけでは初期化・更新の経過を残せない。

次回計測の具体的topic・パラメータ・追加実装要件は `../../README_vio_diagnosis.md` を参照。
最優先は実設定一式・実行版の保存、DEBUG標準出力、save_total_state、trackhist、採択/棄却特徴数・innovation、device timestamp/sequence。

## 成果物

各bagディレクトリ:
- `vio_timeline.png`: VIO/wheel/GNSS速度、z、加速度、gyro、画像追跡代理指標。
- `startup_diagnosis.png`: 初期化前後の姿勢変化とIMU・露光・特徴。
- `startup_frames.png`: 初期化前後の画像。
- `image_quality.png`, `*_contact_sheet.png`: 全走行区間の画像品質・場面。
- `audit.json`, `diagnosis_summary.json`, 各topic CSV、rosout/parameters/static_tf JSON。

8月ディレクトリ追加:
- `startup_replay_comparison.png`, `startup_replay_summary.json`
- `replay_current_config/` / `replay_frozen_calibration/`: odom.csv, state_estimate.txt, state_std.txt, console.log, timing.txt, run.json
- `replay_full_current_config/` / `replay_full_frozen_calibration/`: 全区間対照再生
- `offline_vio_overlay_stable_prefix.png`: 初期安定区間のGNSS・wheel+Wit IMU・校正固定VIO重畳
- `offline_vio_overlay_map.png`, `offline_vio_overlay_full.png`: 全区間の地図範囲表示と発散を含む全範囲表示

公式参考:
- https://docs.openvins.com/gs-calibration.html （校正の運動励起・可観測性）
- https://docs.openvins.com/classov__init_1_1InertialInitializer.html （初期化）
- https://docs.openvins.com/eval-timing.html （timing保存）
