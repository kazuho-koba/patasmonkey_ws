User prompt
ありがとうございます、では推奨のやり方で進めてください。

Codex final response
ステップ4のGNSS拘束型global EKFを実装しました。既存のlegacy構成は既定値のまま残し、`localization_mode:=separated_offroad` の時だけ新構成が選ばれます。

追加した [ekf_global_gnss_constrained.yaml](/home/kazuho/Projects/patasmonkey_ws/src/pm_config/config/ekf_global_gnss_constrained.yaml:1) は次だけを融合します。

- ホイール: 前方速度 `vx`
- Wit IMU: roll/pitch と `wz`。磁気yawの連続直接融合はしません。
- VIO: 健全性ゲート通過後の `vx` / `vy` のみ
- GNSS: `/odometry/gps` の地図座標 `x` / `y` のみ

従来の `/odometry/local` pose はglobal EKFへ入れません。これはlocal EKF内で既に融合済みの相関データであり、GNSS 1 Hzの位置補正を実質的に押し負かす経路をなくすためです。GNSS高度 `z` とVIOの`z`/`vz`は、品質ゲートを実装する次段階まで意図的に除外しています。

[pm_bag_global_localization.launch.py](/home/kazuho/Projects/patasmonkey_ws/src/pm_bringup/launch/pm_bag_global_localization.launch.py:485) と [pm_global_localization.launch.py](/home/kazuho/Projects/patasmonkey_ws/src/pm_bringup/launch/pm_global_localization.launch.py:279) に新しいglobal EKFを追加しました。`/odometry/global` の出力名は維持されます。

実行時は例えば以下です。

```bash
ros2 launch pm_bringup pm_bag_global_localization.launch.py \\
  localization_mode:=separated_offroad
```

この構成でもglobal出力のroll/pitchはWit IMU由来で含まれ、yawはWitの`wz`積分で更新されます。地球基準の初期yawは、前ステップで追加した停止時初期化・校正済み補正値の経路にだけ依存します。

Docker開発環境で `pm_config`、`pm_bringup`、`pm_localization`、`pm_evaluation` のビルド、および両launchの引数・構文確認を完了しました。実機走行では `/odometry/gps` が約1 Hzで出ること、global EKFがGNSS更新ごとにx/yへ収束しつつその間を速度予測で補間することを確認対象にしてください。
