User prompt
===========

OpenVINSを加速度jerkなしでも自動的に開始するようなやり方はできないのでしょうか、閾値をゼロにすればできるのかもしれませんが、その場合どのようなリスクが想定されますか？

Codex final response
====================

可能です。ただし、`init_imu_thresh` をゼロにする方法は使えません。現行OpenVINSでは、動的初期化が無効な状態で `init_imu_thresh <= 0` にすると、設定エラーとしてプロセスが終了します。[InertialInitializerOptions.h](/home/kazuho/ros2_ws/src/open_vins/ov_init/src/init/InertialInitializerOptions.h:159)

現在の設定は以下です。

- `try_zupt: false`
- `init_imu_thresh: 0.2`
- `init_max_disparity: 10.0`
- `init_dyn_use: false`

この構成では、OpenVINSが意図的に「静止状態から動き始める瞬間」を待ちます。なお、ログ上は `no accel jerk detected` ですが、外側のjerk判定には画像特徴のdisparityも使われています。[InertialInitializer.cpp](/home/kazuho/ros2_ws/src/open_vins/ov_init/src/init/InertialInitializer.cpp:133)

推奨できる方法は2つあります。

1. 設定だけで試す方法

   `try_zupt: true`、`zupt_only_at_beginning: true` とし、`init_imu_thresh: 0.2` は維持します。OpenVINSの実装ではZUPTが有効だと `wait_for_jerk=false` になり、約3秒の初期化窓で静止していると判断できれば、その場で静的初期化します。[VioManagerHelper.cpp](/home/kazuho/ros2_ws/src/open_vins/ov_msckf/src/core/VioManagerHelper.cpp:103)

   `zupt_only_at_beginning: true` にするのは、走行開始後まで誤ったゼロ速度拘束を掛け続けるリスクを減らすためです。

2. 実装を少し変更する方法（長期的にはこちらを推奨）

   `init_wait_for_jerk` のような独立パラメータをOpenVINSへ追加し、初期化時だけ `wait_for_jerk=false` にします。これなら、静止中の自動初期化と、走行中のZUPT使用有無を分離できます。現在は「ZUPTが存在するか」でjerk待ちを決めており、この2つが不要に結合されています。

閾値を極端に下げたり、静止確認を弱めたりした場合の主なリスクは次のとおりです。

- 車体加速度を重力と誤認し、初期roll/pitchが傾く
- 旋回中の角速度をgyro biasとして取り込み、以後の姿勢がドリフトする
- 初期速度をゼロと仮定するため、移動中に初期化すると状態が不整合になる
- accelerometer bias推定へ実運動が混入する
- ZUPTを走行中に誤受理すると、速度をゼロへ拘束して軌跡が潰れる
- 低速走行、遠景、特徴不足、カメラ遮蔽では画像disparityが小さく、静止と誤判定する可能性がある

また、静止初期化を許可してもyawは観測できません。重力から決まるのは主にroll/pitchで、初期yaw・地図方位の問題は別途GNSS heading、磁気方位、既知姿勢などで扱う必要があります。

まずは設定だけの比較として、`try_zupt: true`、`zupt_only_at_beginning: true`、`init_imu_thresh: 0.2` で静止起動試験を行うのが妥当です。ただし現在の `zupt_chi2_multipler: 0` は画像disparity主体の判定なので、実車採用前には低速発進を静止と誤判定しないか確認が必要です。安定運用を目指すなら、その試験後に `init_wait_for_jerk` を独立実装するのが最も明確です。
