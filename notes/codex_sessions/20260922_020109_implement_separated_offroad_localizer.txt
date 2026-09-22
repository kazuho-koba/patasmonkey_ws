User prompt

2. local EKFからWit orientationの直接融合を外す というのを次は実行したいですが、これは丸々文字通り実行するのでなく、少し工夫をしてほしいです。
ホイールのvxとIMUのwzを使うというのは平面移動の範囲内だけの話で、このUGVはオフロード走行を前提に設計しているので、local ekfでも車体の姿勢変化や（坂道、段差等による）z方向の移動も加味したいです。
z方向移動はホイールオドメトリからは計算しようがなく、IMUの加速度積分でもうまくいかないとおもうので、VIOの結果を使うしか無いのではないかと思っています。

これらの状況も踏まえて、このステップ２の実行をしてほしいです。実装方針については適宜僕に確認をとって進めてください。
作業は基本的に開発PCのコンテナ環境で、必要に応じてjetsonに移ってください。

この方針でOKです。

いまやってるセッションについて、localekf、globalekfを再計算するなら、その結果のcsvやプロット図なども残すようにしてほしいです。

了解です、ではその方向で進めてください。
終わったら、実装した内容をもう少し詳しく説明する解説もつけてください。
また一段落したらプロンプトと回答の記録もお願いします（これは今回の回答のあとでなくとも、一段落したらでいいです）

Codex final response

実装・オフライン再生検証を完了しました。既存の`legacy`構成は既定値のまま残し、オフロード用に明示選択する`separated_offroad`構成を追加しています。

構成は次のように分離しました。

- 水平EKF: ホイール`vx`、Witの`wz`、健全性を確認済みのVIO `vx/vy`だけを融合します。`two_d_mode: true`なので、VIOのz破綻やroll/pitchがx/y/yawへ相関して汚染する経路はありません。
- 姿勢・高さ観測器: Wit orientationからroll/pitchのみを取り、Wit yawは使いません。高さは厳格ゲートを通過したVIO `z`だけを採用し、VIOがリセット・発散すると「最後に妥当だったz」を保持します。IMU加速度の二重積分は使いません。
- composer: 上の水平`x/y/yaw`と姿勢・高さ`roll/pitch/z`を結合し、従来互換の`/odometry/local`と`odom -> base_link` TFを出力します。global EKF / navsat_transformの入力インターフェースは維持されます。

単一の3D EKFへVIO zを直接入れる試作は採用しませんでした。VIOが停止した後も、3D EKFの推定済み`vz`が予測で漂い続けることを7月・8月bagで確認したためです。現在の観測器は、VIO異常時に高さを勝手に積分しないことを優先しています。

VIO保護は二段です。`vio_vertical_gate_node`は非有限値、速度>2.5 m/s、`|vz|`>1.0 m/s、z共分散、1フレーム当たりz変化>0.25 mを検査し、異常後はラッチ閉鎖します。`vio_twist_gate_node`は水平EKFに渡す相対速度を検査し、少なくとも4.0秒間連続して数値的に健全なVIOだけを再投入します。これはOpenVINS再初期化直後の一見有限だが誤った速度を避ける待ち時間です。2.5 m/sは車両最大2 m/sと正常だった7月VIOのピーク約2.08 m/sの両方を踏まえた余裕値です。なお再投入は「数値的な安定」を保証するだけで、OpenVINSの意味論的な健全性を完全に保証するものではありません。今後記録するOpenVINS診断と組み合わせて強化できます。

主な実装は[水平EKF設定](/home/kazuho/Projects/patasmonkey_ws/src/pm_config/config/ekf_local_horizontal_vio_twist.yaml)、[VIO速度ゲート](/home/kazuho/Projects/patasmonkey_ws/src/pm_localization/pm_localization/vio_twist_gate_node.py)、[VIO高さゲート](/home/kazuho/Projects/patasmonkey_ws/src/pm_localization/pm_localization/vio_vertical_gate_node.py)、[姿勢・高さ観測器](/home/kazuho/Projects/patasmonkey_ws/src/pm_localization/pm_localization/attitude_height_observer_node.py)、[odometry合成器](/home/kazuho/Projects/patasmonkey_ws/src/pm_localization/pm_localization/local_odometry_composer_node.py)です。両方のlaunchに`localization_mode:=separated_offroad`を追加し、bag launchには中間出力`/odometry/local_horizontal`、`/odometry/local_vertical`と各ゲート診断も記録対象として追加しました。

再生結果です。

- 7月bag: 最終水平位置は `(6.39, -1.93) m`、高さは`-0.799 m`。VIOのzが1.727 m段差でリセットした時点で高さゲートが閉鎖しました。速度ゲートは4秒の安定化後に再投入し、後段の速度不連続（1.902 m/s）で隔離してから、再び4秒の安定化を要求しました。
- 8月bag: 旧local EKF/VIOは十万m級へ発散しますが、新構成の最終水平位置は `(80.36, -0.33) m`、高さは`0.094 m`、高さ範囲は0.112 mです。高さゲートはVIO速度2.501 m/sで閉鎖し、速度ゲートは4秒の安定化を満たす前に異常となったため、破綻したVIO速度を水平EKFへ渡しませんでした。

可視化・CSV・再生MCAPは、[7月の安全出力プロット](/home/kazuho/Projects/patasmonkey_ws/src/pm_evaluation/results/separated_offroad_localization/rosbag2_2026_07_26-09_17_38_stable4s_rate10/separated_localization_safe_output.png) と [summary.json](/home/kazuho/Projects/patasmonkey_ws/src/pm_evaluation/results/separated_offroad_localization/rosbag2_2026_07_26-09_17_38_stable4s_rate10/summary.json)、[8月の安全出力プロット](/home/kazuho/Projects/patasmonkey_ws/src/pm_evaluation/results/separated_offroad_localization/rosbag2_2026_08_10-18_42_30_stable4s_rate10/separated_localization_safe_output.png) と [summary.json](/home/kazuho/Projects/patasmonkey_ws/src/pm_evaluation/results/separated_offroad_localization/rosbag2_2026_08_10-18_42_30_stable4s_rate10/summary.json) にあります。各ディレクトリには`composed_local.csv`、`horizontal_ekf.csv`、`attitude_height_observer.csv`、入力VIO CSV、replayed MCAP、旧推定との比較図も残しています。新しい[再生CLI](/home/kazuho/Projects/patasmonkey_ws/src/pm_evaluation/pm_evaluation/cli/replay_separated_localization.py)は、これらを再生成します。

8月の約80 mは「絶対位置精度がGNSSと一致した」という意味ではありません。VIOを隔離した後の水平推定は主にホイール＋Wit `wz`のデッドレコニングなので、GNSSとの定量比較は次の実走bagで行うべきです。今回確認できたのは、VIO破綻がlocal EKF全体を十万m級に破壊しない、という安全性です。

実機では、ビルド・source後に次で新構成を選択できます。

```bash
ros2 launch pm_bringup pm_bag_global_localization.launch.py \
  localization_mode:=separated_offroad
```

検証は開発コンテナで行い、`pm_config`、`pm_localization`、`pm_bringup`、`pm_evaluation`のビルド成功、両launchの`--show-args`による非起動構文確認、新規Pythonの`py_compile`、`git diff --check`を確認済みです。Jetsonには変更していません。
