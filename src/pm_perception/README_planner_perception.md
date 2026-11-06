# planner向け独立画像評価node

`planner_mapper_node` は撮像時刻のTF・既存のdepth入力処理を再利用し、
画像ごとに地形を計算してXYセルへ結果を保持する試験実装です。
`pm_core`の既定producerもこのnodeです。従来の高さ時間融合nodeは比較用に残しています。
UGV通常運用の設定は`pm_config/config/planner_perception.yaml`です。
このパッケージの`config/planner_perception.yaml`は下記の単独試験launchだけが既定で使います。
両ファイルは用途ごとに独立して変更でき、自動同期はしません。

GUIの「障害物回避ON」はこのnodeのobstacle配列、「障害物回避＋路面評価ON」は
obstacleとterrainの両配列をplannerへ接続します。認識modeの初期値は従来どおりOFFで、
producerの起動だけでは自律走行を開始しません。

```bash
ros2 launch pm_perception planner_perception.launch.py
# bagでは時計を合わせる
ros2 launch pm_perception planner_perception.launch.py use_sim_time:=true
# 設定をコピーして編集した場合
ros2 launch pm_perception planner_perception.launch.py params_file:=/absolute/path/settings.yaml
```

先にROS、ros2_ws、patasmonkey_wsの順でsourceしてください。
従来の`depth_elevation_mapper`と同時起動しないでください。
depth、CameraInfo、撮像時刻のcamera→odomとbase→odomのTFが必要です。
旧bagでCameraInfoがない場合は、適合する校正値を確認してから設定してください。

## 出力と判定

- `/depth_elevation_mapper/planner_costmap`: `pm_msgs/TerrainAssessment`。
  obstacleとterrainを別配列で送ります。
- `/depth_elevation_mapper/obstacle_costmap`: RViz用OccupancyGrid。
- `/depth_elevation_mapper/terrain_costmap`: slope・roughness・stepのOccupancyGrid。

値はunknown=-1、危険=100です。初回高点からobstacle=100です。
確認回数不足を理由に危険を保留しません。通常起動と単独試験の設定先は上記のとおりです。

解除は直近N枚の受理画像に対応する窓で行います。有効な低い再観測は
セル内最大の平面相対高さとして保持し、欠測は数値0にせず欠測として保持します。
窓内の有効値の平均が閾値を下回ると解除し、その時点の低い観測も有効なら0、
有効でなければunknownへ戻します。窓全体が欠測の場合もunknownへ戻します。
新しい高点があれば窓を初期化し、直ちに100へ戻します。
画像が来ない場合やTF不成立で画像を処理できない場合は、窓は進みません。

この方式は物体が消えたことの証明ではありません。遮蔽・疎な観測でも
記録を解除し得ます。unknownを安全値として走行に使わないことが前提です。
実機性能・障害物見落とし・走行安全性は未検証です。

GUIからの主要認識parameterの実行中変更に対応しています。変更時はmapを消去します。
`~/reset_map`（`std_srvs/Trigger`）で観測mapだけを消去することもできます。
センサ・local EKF・表示用MPPI・GUI・RVizの一括起動は
`ros2 launch pm_bringup perception_lab.launch.py`です。Joy操縦を行う場合は`drive:=true`を指定します。
