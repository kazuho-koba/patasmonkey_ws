# pm_bringup：記録付き自己位置推定launch

`pm_bag_global_localization.launch.py`は、UGVの通常センサ起動とrosbag記録を
まとめた検証用launchです。既定の`legacy`構成を維持し、オフロード用の新構成は
`localization_mode:=separated_offroad`で明示選択します。

```bash
ros2 launch pm_bringup pm_bag_global_localization.launch.py \
  localization_mode:=separated_offroad \
  bag_name:=rosbag2_test_name
```

このモードでは、水平EKF・姿勢/高さ観測器・VIOゲート・heading初期化器・GNSS品質
ゲート・GNSS拘束global EKFを起動します。`/odometry/local`と`/odometry/global`の
topic名、`odom -> base_link` TFは従来互換です。

bagには中間出力`/odometry/local_horizontal`、`/odometry/local_vertical`、VIO各
ゲート診断、`/fix`、`/fix/gated`、`/navpvt`、`/gnss/fix_gate/diagnostics`も記録
します。生データとゲート後データを同一試行で比較できることが目的です。

起動後は、停止時heading初期化が完了してから走行してください。GNSSが低品質の
とき`/fix/gated`や`/odometry/gps`が出ないのは安全側の正常な挙動であり、まず
`/gnss/fix_gate/diagnostics`の理由を確認します。

OpenVINS・OAK-D側の追加記録内容は[README_vio_recording.md](README_vio_recording.md)、
localizationの設計は[pm_localization README](../pm_localization/README.md)を参照してください。
