User prompt:
stage3に行く前に、rviz2でロボットモデルを原寸大表示できないでしょうか。サイズ感がわかりにくいです。

Codex final response:
対応しました。最新localizationでbagを再計算するlaunchを起動すると、RVizに`Patasmonkey model (actual scale)`が有効な状態で表示されます。

URDFのmeshは既存の`pm_description/urdf/pm.urdf`をそのまま使っており、mesh scaleも含めて原寸です。モデルは、最新localizerがpublishする`odom -> base_link`へ追従します。

通常どおり以下で確認できます。

```bash
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
source /workspaces/patasmonkey_ws/install/setup.bash
export ROS_DOMAIN_ID=91
ros2 launch pm_perception terrain_mapping_latest_localization_replay.launch.py
```

別terminalでbag player、さらにRVizを起動してください。RViz左側のDisplaysに`Patasmonkey model (actual scale)`があります。不要なら同displayのEnabledを外せます。

このreplay launch内の`robot_state_publisher`は、RobotModel用の`/robot_description`と車輪などのjoint TFを供給します。一方、固定TFはprivate topicへremapしたため、bagから再生する`/tf_static`と競合しません。従来どおりbagの`/tf`は再生せず、current localizerだけが`odom -> base_link`をpublishします。

変更したファイルは、`src/pm_perception/launch/terrain_mapping_latest_localization_replay.launch.py`、`src/pm_perception/rviz/terrain_mapping.rviz`、`src/pm_perception/package.xml`、`src/pm_perception/README.md`です。Foxyコンテナで`pm_perception`を再ビルドし、launch定義の読み込みも確認済みです。
