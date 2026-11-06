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

## 保持範囲と旋回

セルは引き続きodom軸固定で、絶対XYセルごとに履歴を保存する。
新方式では「どのセルを保持するか」という範囲だけを撮像時刻のbase位置・yawへ追従させる。
高さや履歴を回転・補間せず、範囲内に残る同じセルの状態を引き継ぐ。

|parameter|設定ファイルの既定値|意味|
|---|---|---|
|`map_retention_mode`|`body_aligned`|車体の向きに追従。`odom_aligned`なら従来方式|
|`map_forward_offset_m`|`2.0`|保持正方形の中心を車体前方へずらす距離[m]|
|`map_size_x/y`|`8.0`|保持正方形の辺長[m]。両値は同じにする|

この設定は`pm_config/config/planner_perception.yaml`（Coreとlab両モード）と、
`pm_perception/config/planner_perception.yaml`（単独試験）の両方へ設定している。
他の既存値は用途別のまま維持する。YAMLを渡さないnode/APIの既定値は互換性のため
`odom_aligned`・offset=0のままなので、新方式には設定ファイルを使用する。

新方式の8m・offset=2mでは、どのyawでも車体基準で前方約6m・後方約2m・左右約4mとなる。
offsetは正が前方、負が後方で、絶対値を半辺以下に制限する。
セル中心で半開区間の採否を決めるため、境界はセル幅程度の離散化を伴う。
`odom_aligned`ではoffsetは適用せず、従来と同じ車体中心・odom軸固定の正方形となる。

```text
          車体前方
          ↑
    ┌───────────┐
    │  前方6m   │  ← この保持範囲だけ車体と共に移動・旋回
    │           │
    │    UGV    │
    │  後方2m   │
    └───────────┘
      左右各4m
  セルのXY座標軸は常にodom固定
```

OccupancyGridは従来と同じodom軸固定・原点姿勢identityのまま配信する。
斜めの正方形を包む外接矩形なので、**配信width/heightと原点はyaw・位置によって変わる**。
外接矩形の角部分はunknownであり、8m四方より広く保持しているわけではない。
受信側は毎messageのMapMetaDataを使用する。現行plannerとlabはこの形式に対応する。
`~/retention_boundary`のMarkerをlabのRVizへ橙色で表示し、実際の保持範囲を区別する。

固定の最大サイズを毎回処理せず、各時点の外接矩形だけを計算する。
45度では辺長が最大約11.4mとなり、一時配列・出力セル数は従来の約２倍になる。
内部の履歴は保持範囲内のセルだけで、全地図の再サンプリングはしない。
旋回して範囲外になった観測は破棄し、戻っても復元しない。画像／TFが来なければ範囲も更新しない。
カメラの視野角やdepth上限、有効評価の条件は変更していないため、前方6mが全て既知になる保証ではない。

オンライン・オフライン共通GUIの「map保持範囲の向き」と「保持範囲の前方offset」で変更できる。
「適用」で既存mapをリセットする。「cost切替・予測設定だけ適用」はこの設定を変更しない。
保存する認識YAMLにも２項目を含む。CLIで従来方式へ戻す場合：

```bash
ros2 param set /depth_elevation_mapper map_retention_mode odom_aligned
# 新方式へ切替（offsetは設定ファイルの2mを維持）
ros2 param set /depth_elevation_mapper map_retention_mode body_aligned
```

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
