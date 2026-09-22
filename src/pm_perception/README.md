# pm_perception Stage 1: depth elevation mapping

OAK-D S2のaligned depthをPointCloud2へ展開せず、直接`odom`座標の
robot-centric rolling 2.5D gridへ融合する最小実装です。traversability、
Nav2 cost、semantic処理はまだ含みません。

## 調査結果（2026-09-22）

- depth: `/oak/depth/image_raw`、`sensor_msgs/Image`、`16UC1` (mm)、640x400。
  driver設定値はRGB-D 10 Hzで、bag実測も約10.01 Hzでした。
- depthはRGBへalign済みです。現行driverにCameraInfoが無かったため、
  `/oak/depth/camera_info`をEEPROMからpublishする変更を`depthai_driver`へ追加しました。
- 既存bagのdepth frameは`oak_rgb_camera_optical_frame`ですが、URDFのTF名は
  `rgb_camera_optical_frame`です。旧bagでは専用launchがframe名をoverrideします。
  新しいdriverはURDF側の`rgb_camera_optical_frame`を既定値にしました。
- static TFは`base_link -> oakd_link -> rgb_camera_link ->
  rgb_camera_optical_frame`です。URDFの0.28 mはシャシ底面である`base_link`から
  レンズまでの高さであり、最低地上高0.12 mを加えた暫定の対地高さは0.40 mです。
  静的TFは正しい座標系の関係を表すため0.28 mのまま維持します。0.40 mは
  `nominal_camera_height_above_ground`として、後続Stageでground-relativeな処理を
  導入する際の基準値にします（実測で更新予定）。
- legacy localizationの`ekf_local_node`が`odom -> base_link`を50 Hzでpublishし、
  x/yはwheel/VIO速度、zはVIO速度、roll/pitch/yawはHWT905姿勢を融合します。
- `separated_offroad`構成ではhorizontal EKFがx/y/yaw、attitude/height observerが
  HWT905 roll/pitchとgate済みVIO zを持ち、composerが6DoFの
  `odom -> base_link`をpublishします。VIO heightが無い間はz=0、異常時は最後の
  妥当値を保持するため、完全に独立観測された6軸ではありません。
- 代表bagではdepthの全時間範囲を50 Hzの`odom -> base_link` TFが覆っており、
  `/tf`、`/tf_static`、`/odometry/local`、depthは記録済みです。CameraInfoだけは
  過去bagにありません。

## 処理とデータ構造

1. CameraInfo（旧bagでは明示したEEPROM fallback）を取得。
2. `pixel_stride`間隔でNumPy viewを取り、0値とdepth範囲外を除去。
3. pinhole modelでoptical frameへ一括back-project。
4. depth headerの**撮像時刻**でcamera→odomとbase_link→odomをlookup。TFがまだ
   到着していないframeは短いbounded queueで待ち、timeout後はdropします。
   latest TFへのfallbackはありません。
5. NumPy行列演算でodomへ変換し、XYセルごとの最小/最大zを`minimum.at`/
   `maximum.at`で集約。
6. 最小zをground候補として、距離依存のdepth分散から求めた逆分散重みで逐次融合。
   古い観測の重みは指数的に減衰しますが、rawな観測数は診断用に保持します。
7. 同じground候補を、`camera_z - nominal_camera_height_above_ground`を基準に
   正規化した`relative_elevation`にも融合します。これはodomの共通zずれを抑えて
   平坦地を概ね0 mで表示するための診断レイヤです。
8. 大きく高いreturnはground平均へ混ぜず、obstacle height/confidence fieldへ蓄積。

rolling gridは固定長arrayと絶対odom cell tagを用いたring bufferです。車両移動時に
全mapをコピーせず、map外へ出たslotを再利用時にlazy clearします。内部layerは
`elevation`、その重み付き分散、`observation_count`、`last_observed_ns`、
`relative_elevation`に加え、`obstacle_height`、`obstacle_confidence`、
`last_obstacle_observed_ns`です。

## build

Foxy container内でoverlay順を守ってbuildします。

```bash
docker exec -it --user 1000:1000 --env HOME=/home/developer \
  patasmonkey_foxy_dev bash
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
cd /workspaces/patasmonkey_ws
colcon build --packages-select pm_perception
```

`depthai_driver`変更を反映する場合は、別途`ros2_ws`をbase Foxyだけsourceしてbuildします。

## live実行

```bash
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
source /workspaces/patasmonkey_ws/install/setup.bash
ros2 launch pm_perception terrain_mapping.launch.py
```

既存bringupへは自動追加していません。まず独立起動で負荷とTFを確認してください。

## 旧rosbagでの確認

以下の`terrain_mapping_old_bag.launch.py`は、記録済みの`/tf`をそのまま用いる
**Stage 1互換の簡易確認用**です。最新localizationでStage 2を検証する場合は、必ず
次節の再計算手順を使ってください。

terminal 1:

```bash
ros2 launch pm_perception terrain_mapping_old_bag.launch.py
```

terminal 2:

```bash
ros2 bag play /workspaces/patasmonkey_ws/bags/<bag> -s mcap \
  --topics /oak/depth/image_raw /oak/color/image_raw /tf /tf_static
```

terminal 3（任意）:

```bash
rviz2 -d $(ros2 pkg prefix pm_perception)/share/pm_perception/rviz/terrain_mapping.rviz
```

`/depth_elevation_mapper/elevation_points_debug`を有効にした旧bag設定では、実高度の点を
odom固定で確認できます。前方で得た点がcamera FOV外になっても、6 m window内なら同じ
odom位置に残ることを、RViz Fixed Frame=`odom`で確認してください。
OccupancyGridは高さを`debug_elevation_min..max`から0..100へ色変換した表示専用品で、
Nav2 costではありません。

既存bagを編集する必要はありません。以下の2 bagはどちらもdepth、color、`/tf`、
`/tf_static`を含み、上の旧bag launchで閲覧できます（CameraInfoは未収録なので
EEPROM fallbackを使います）。

```bash
# 2026-07-26 bag
ros2 bag play /workspaces/patasmonkey_ws/bags/rosbag2_2026_07_26-09_17_38 \
  -s mcap --topics /oak/depth/image_raw /oak/color/image_raw /tf /tf_static

# 2026-08-10 bag
ros2 bag play /workspaces/patasmonkey_ws/bags/rosbag2_2026_08_10-18_42_30 \
  -s mcap --topics /oak/depth/image_raw /oak/color/image_raw /tf /tf_static
```

RViz設定の`OAK color` displayは`/oak/color/image_raw`を表示します。表示を消したい
場合や別画像topicを比較したい場合は、左側`Displays`から同displayの`Enabled`または
`Topic`を変更してください。

同梱RViz設定はOrbit viewの`Target Frame`を`base_link`に設定済みです。Fixed Frameは
`odom`のままなので、mapはodom固定、カメラの注視点だけがロボットに追従します。

## 7月bagを現在のlocalizationで再計算して可視化する

`terrain_mapping_latest_localization_replay.launch.py`は、現在の
`separated_offroad`相当のlocalizer（horizontal EKF、VIO gate、attitude/height
observer、composer）を起動します。このlaunchが新しい`/odometry/local`と
`odom -> base_link` TFを唯一publishします。

そのため、replay側には**記録済みの`/tf`を含めません**。旧`odom -> base_link`と
再計算TFを同時に流すと、TFが競合してterrain mapを検証できなくなります。

terminal 1:

```bash
export ROS_DOMAIN_ID=91  # 他の実機/ROS実行系から分離する任意のdomain
ros2 launch pm_perception terrain_mapping_latest_localization_replay.launch.py
```

terminal 2:

```bash
export ROS_DOMAIN_ID=91
ros2 run pm_evaluation bag_clock_player \
  /workspaces/patasmonkey_ws/bags/rosbag2_2026_07_26-09_17_38 \
  --rate 1.0 \
  --topic /wheel/odometry \
  --topic /vio/odometry \
  --topic /wit/imu \
  --topic /tf_static \
  --topic /oak/depth/image_raw \
  --topic /oak/color/image_raw
```

terminal 3:

```bash
export ROS_DOMAIN_ID=91
rviz2 -d "$(ros2 pkg prefix pm_perception)/share/pm_perception/rviz/terrain_mapping.rviz"
```

このlaunchは`pm_description/urdf/pm.urdf`を`robot_state_publisher`へ渡すため、RVizの
`Patasmonkey model (actual scale)`が`odom -> base_link`に追従して原寸で表示されます。
モデルの固定TFはprivate topicにremapしており、再生する`/tf_static`とは競合しません。
表示が不要なら、RViz左側のDisplaysで同displayを無効にしてください。

このplayerはMCAPをstreamするため、RGB-D message全体をメモリへ保持しません。
これは新しいlocalizationのオフライン再計算結果をその場でterrain mapperへ渡す可視化用であり、
元bagや既存の解析結果を変更しません。

### Stage 2で確認する表示

上記のterminal 3でRVizを開き、左側の`Displays`から必要なMap displayを有効にします。
すべてdebug用の線形グレースケールであり、Nav2 costではありません。

| topic | 0（黒）→100（白）の意味 | 確認すること |
|---|---|---|
| `/depth_elevation_mapper/elevation_debug` | odom-zの低値→高値 | 従来の絶対odom elevation |
| `/depth_elevation_mapper/relative_elevation_debug` | -0.30→+0.30 m | 平坦面が概ね中間値（0 m）になるか。odom-zだけが動く場合との違い |
| `/depth_elevation_mapper/elevation_variance_debug` | 低分散→`debug_variance_max` | 白いセルほど不確か。遠距離・ばらつきのある箇所を確認 |
| `/depth_elevation_mapper/observation_count_debug` | 0→10観測（既定） | 同じodomセルに複数frameが融合されているか |
| `/depth_elevation_mapper/observation_age_debug` | 新しい→5秒以上前 | FOV外になった観測がrolling window内に残りつつ古くなること |
| `/depth_elevation_mapper/obstacle_height_debug` | 0→0.50 m | ground候補より上のreturn。低confidence/未観測はunknown |

比較時は、平坦な区間で`relative_elevation_debug`が極端に変化しないこと、2 m前方の
groundが接近後も`observation_age_debug`で残ること、岩や草でobstacle layerが過剰に
反応しないことを順に確認してください。

## 主なparameter

| parameter | default | 意味 |
|---|---:|---|
| `map_size_x/y` | 6.0 m | rolling window |
| `resolution` | 0.05 m | cellサイズ（120x120） |
| `min_depth/max_depth` | 0.4/4.0 m | 使用depth |
| `nominal_camera_height_above_ground` | 0.40 m | 暫定カメラ対地高さ。Stage 2の`relative_elevation`基準。静的TFやodom-zは変更しない |
| `pixel_stride` | 4 | 4x4 sampling |
| `max_processing_rate` | 12 Hz | frame rate上限（現行10 Hz入力をjitterで間引かない） |
| `tf_wait_timeout` | 0.25 s | exact timestamp TF待ち |
| `ground_merge_threshold` | 0.20 m | ground仮説へ融合するz差 |
| `measurement_variance` | 0.0025 m² | 簡易観測分散floor |
| `depth_variance_per_meter_sq` | 0.0004 m²/m² | 遠距離sampleの分散増分。`floor + coefficient × range²` |
| `observation_decay_time` | 8 s | stale観測の融合重み・obstacle confidenceの時定数 |
| `obstacle_confidence_min` | 0.15 | obstacle debug表示の最小confidence |
| `publish_debug_occupancy` | true | 高さ色表示をpublish |
| `publish_stage2_debug_layers` | true | relative / variance / count / age / obstacleのdebug topicをpublish |
| `publish_debug_pointcloud` | false | 1点/cellのdebug cloud |

## 現段階の制約と次段候補

- カメラ取付高さ/姿勢とRGB optical extrinsicは暫定値です。最優先で実測・校正します。
- 最小z方式はvegetation、overhang、negative obstacleを十分分類できません。
- `relative_elevation`は各撮像時刻の暫定カメラ対地高さで正規化する診断レイヤです。
  車両が長い斜面を移動する場合の一貫した世界地形高さではなく、traversabilityへ直結しません。
- 分散モデルは保守的な経験則であり、OAK-Dの距離・反射率・照明ごとの実測校正は未実施です。
- 最小zをground候補とするため、vegetationやnegative obstacleの誤分類はまだ残ります。
- 古いbagのfallback intrinsicsは特定MX ID用です。別個体には適用しません。
- primary map message APIは未固定で、現在のROS出力はdebugのみです。

Stage 2では、距離依存depth分散、時刻aging、relative elevation、最小限の
ground/obstacle二仮説まで実装済みです。次のStage 3では、(1)局所平面を使うground候補の
堅牢化、(2)vertical bandsまたはnegative-obstacle cue、(3)slope/roughness/step layer、
(4)軽量な明示的map message、の順が妥当です。Nav2 cost変換はそれらの後に行います。
