# pm_perception Stage 3: depth elevation mapping and local terrain cues

OAK-D S2のaligned depthをPointCloud2へ展開せず、直接`odom`座標の
robot-centric rolling 2.5D gridへ融合する最小実装です。Stage 3ではその
融合済みgridからslope、roughness、step height、暫定hazardを軽量に可視化します。
これらはまだtraversabilityの完成版でもNav2 costでもありません。

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
9. Stage 3.1ではdebug publish時だけ、freshな`relative_elevation`の3x3近傍へ
   閉形式の最小二乗平面をfitします。slopeは平面勾配、roughnessは平面残差RMS、
   step heightは平面残差のmax-minです。stepはさらに車両headingの前後双方に
   観測supportがある場合だけ有効です。傾斜・roughness・step・既存obstacle cueの
   最大正規化値を`terrain_hazard_debug`として表示します。各cueが得られたセルだけを
   個別に合成するため、planeが未計算でもobstacle cueはhazardへ反映されます。欠測や
   古い観測をnegative obstacleとは扱いません。

### Stage 3.1 の4指標の計算方法

以下は現在の実装そのものです。入力は`relative_elevation`（各撮像時刻の暫定カメラ
対地高さで正規化したground候補）であり、元のdepth点群やNav2 costを直接使いません。
各セルでは、観測ageが`feature_max_observation_age`以下で有限値のセルだけを
**fresh**とします。既定では中心セルを含む3x3（cell幅0.05 mなので0.15 m四方）の
近傍で、中心がfresh、freshセル数が`feature_min_neighbors`以上（既定5）、かつ
点配置が一直線でない場合にのみ局所平面を計算します。条件を満たさない値は0ではなく
unknownです。

| 指標 | 現在の計算 | 有効になる条件 |
|---|---|---|
| slope | 近傍の高さに最小二乗平面 `z = a x + b y + c` をfitし、`atan(sqrt(a²+b²))` を度へ変換 | 上記の局所平面がfitできること。上り/下りの符号は持たず、絶対的な勾配角だけを出す |
| roughness | 同じ平面への残差 `r_i = z_i - (a x_i + b y_i + c)` のRMS、`sqrt(mean(r_i²))` | 局所平面がfitできること。滑らかな傾斜面なら小さく、石・草・depthノイズ等で大きくなる |
| step height | 同じ残差のpeak-to-peak、`max(r_i) - min(r_i)` | 局所平面に加え、robot heading方向の前側と後側に、それぞれ`step_min_side_neighbors`個以上のfreshセルがあること。headingは「観測supportのgate」にだけ使い、現時点の段差量自体は近傍全体の残差範囲 |
| obstacle height | 1 depth frame内の同一XY cellの最小zを暫定ground、最大zとの差を候補高さとする。候補が`obstacle_min_height`以上なら、過去の最大高さとconfidenceを保持する。後続frameで従来groundより`ground_merge_threshold`を超えて低いzが出た差も候補として蓄積する | confidenceが`obstacle_confidence_min`以上。confidenceは観測間隔および未再観測時間で指数減衰する。ground/物体の意味的分類ではない |

したがってroughnessとstepは、以前の「生の近傍高さの標準偏差／max-min」ではありません。
平坦でないが滑らかな坂はslopeへ現れ、局所平面から外れた凹凸だけがroughnessとstepへ
現れる設計です。一方、obstacleは局所平面fitから独立した、最小z ground仮説に基づく
軽量な垂直extent cueです。vegetation、縁石、壁、穴の縁、depthの外れ値を確実に区別する
ものではありません。

`terrain_hazard_debug`は、得られた各cueをそれぞれのlimitで割った値の最大です。

```
hazard = clamp(max(slope / hazard_slope_limit_deg,
                   roughness / hazard_roughness_limit,
                   step / hazard_step_limit,
                   obstacle_height / hazard_obstacle_height_limit), 0, 1)
```

存在しないcueはmaxから除外します。複数cueが同じ最大値の場合、debug用
`terrain_hazard_cause_debug`は実装上 `slope → roughness → step → obstacle` の順で最初の
ものを表示します。これは最大寄与の表示であり、他のcueが閾値を超えていないことを保証
しません。いずれも暫定的なterrain cueで、走行可否・Nav2 cost・negative obstacle検出を
意味しません。

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
  --topic /oak/depth/camera_info \
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

### 任意の対応bagでStage 3を可視化する手順

最新localizationで再計算した結果を使う場合、bagが少なくとも
`/wheel/odometry`、`/vio/odometry`、`/wit/imu`、`/tf_static`、
`/oak/depth/image_raw`を含むことを確認します。CameraInfoを含む新しいbagでは
`/oak/depth/camera_info`も渡してください（旧bagではこの行を外し、既存fallbackを使用）。
bagに記録済みの`/tf`は再生しません。

terminal 1:

```bash
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
source /workspaces/patasmonkey_ws/install/setup.bash
export ROS_DOMAIN_ID=91
ros2 launch pm_perception terrain_mapping_latest_localization_replay.launch.py
```

terminal 2 (`<bag_directory>`を任意のbag directoryへ置換):

```bash
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
source /workspaces/patasmonkey_ws/install/setup.bash
export ROS_DOMAIN_ID=91
ros2 run pm_evaluation bag_clock_player <bag_directory> --rate 1.0 \
  --topic /wheel/odometry --topic /vio/odometry --topic /wit/imu \
  --topic /tf_static --topic /oak/depth/image_raw \
  --topic /oak/depth/camera_info --topic /oak/color/image_raw
```

terminal 3:

```bash
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
source /workspaces/patasmonkey_ws/install/setup.bash
export ROS_DOMAIN_ID=91
rviz2 -d "$(ros2 pkg prefix pm_perception)/share/pm_perception/rviz/terrain_mapping.rviz"
```

RViz左側の`Displays`で`Elevation debug`を基準に、`Slope (Stage 3)`、
`Roughness (Stage 3)`、`Step height (Stage 3)`、`Preliminary terrain hazard
(Stage 3)`を一つずつ有効化します。Fixed Frameは`odom`のまま、Orbit cameraの
Target Frameは`base_link`なので、地図はodom上に残り、視点だけロボットに追従します。
`Observation age (Stage 2)`を同時に表示すると、FOV外になった地面がrolling window内に
残っていることと、Stage 3が3秒より古いセルをunknownに戻すことを比較できます。

RViz設定の`map` paletteでは、0は白、100は黒、-1（unknown）は灰色です。したがって
`terrain_hazard_debug`の黒は「現在の暫定閾値のいずれかを超えた」という可視化上の意味だけで、
走行禁止・Nav2 cost・negative obstacleの検出結果ではありません。平坦地でこの表示が広く
黒い場合は、まず`relative_elevation_debug`、`roughness_debug`、`observation_age_debug`を
並べて、odomの残留誤差、depthばらつき、古いセルのいずれが主因かを確認してください。

原因を空間的に確認したいbag review時だけ、YAMLで
`publish_hazard_cause_markers: true`にしてbuild/restartし、RVizの`Hazard cause
(colored, optional)`を有効化します。黒hazardセルだけを最大2500点に間引いたcubeで表示し、
赤=slope、緑=roughness、黄=heading方向step、紫=obstacleです。これはmessage生成を抑えるため
既定OFFであり、実機常時runtimeでは有効にしません。

### Stage 2で確認する表示

上記のterminal 3でRVizを開き、左側の`Displays`から必要なMap displayを有効にします。
すべてdebug用の線形グレースケールであり、Nav2 costではありません。

| topic | RVizでの白（0）→黒（100）の意味 | 確認すること |
|---|---|---|
| `/depth_elevation_mapper/elevation_debug` | odom-zの低値→高値 | 従来の絶対odom elevation |
| `/depth_elevation_mapper/relative_elevation_debug` | -0.30→+0.30 m | 平坦面が概ね中間値（0 m）になるか。odom-zだけが動く場合との違い |
| `/depth_elevation_mapper/elevation_variance_debug` | 低分散→`debug_variance_max` | 白いセルほど不確か。遠距離・ばらつきのある箇所を確認 |
| `/depth_elevation_mapper/observation_count_debug` | 0→10観測（既定） | 同じodomセルに複数frameが融合されているか |
| `/depth_elevation_mapper/observation_age_debug` | 新しい→5秒以上前 | FOV外になった観測がrolling window内に残りつつ古くなること |
| `/depth_elevation_mapper/obstacle_height_debug` | 0→0.20 m | ground候補より上のreturn。0.20 m以上は黒へ飽和。低confidence/未観測はunknown |
| `/depth_elevation_mapper/slope_debug` | 0→20° | freshな3x3近傍への最小二乗平面の勾配。20°以上は黒へ飽和 |
| `/depth_elevation_mapper/roughness_debug` | 0→0.03 m | freshな3x3局所平面への残差RMS。3 cm以上は黒へ飽和。草、石、depthのばらつきにも反応する |
| `/depth_elevation_mapper/step_height_debug` | 0→0.07 m | heading前後双方のsupportを満たしたfreshな3x3局所平面残差のmax-min。7 cm以上は黒へ飽和 |
| `/depth_elevation_mapper/terrain_hazard_debug` | 0→1の暫定正規化値 | slope / roughness / step / obstacle cueの最大値。**Nav2 costではない** |
| `/depth_elevation_mapper/terrain_hazard_cause_debug` | 1=slope, 2=roughness, 3=step, 4=obstacle | 最大寄与cueの数値code。主にbag解析用 |

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
| `obstacle_min_height` | 0.20 m | 1 frame内の垂直extentをobstacle候補として蓄積し始める最小高さ |
| `obstacle_confidence_min` | 0.15 | obstacle debug表示の最小confidence |
| `publish_debug_occupancy` | true | 高さ色表示をpublish |
| `publish_stage2_debug_layers` | true | relative / variance / count / age / obstacleのdebug topicをpublish |
| `publish_stage3_debug_layers` | true | slope / roughness / step / provisional hazardのdebug topicをpublish |
| `feature_max_observation_age` | 3.0 s | これより古いセルはStage 3 featureをunknownにする |
| `feature_neighborhood_radius_cells` | 1 | feature近傍半径（既定は3x3） |
| `feature_min_neighbors` | 5 | roughness/stepを計算する最低fresh観測数 |
| `step_min_side_neighbors` | 2 | headingの前方・後方それぞれでstepに必要なfresh観測数。不足時はunknown |
| `hazard_slope_limit_deg` | 20° | 暫定hazard=1となるslopeの目安。走行可否閾値ではない |
| `hazard_roughness_limit` | 0.03 m | 暫定hazard=1となるroughnessの目安 |
| `hazard_step_limit` | 0.07 m | Patasmonkeyの暫定走行不能段差。これ以上でhazard=1 |
| `hazard_obstacle_height_limit` | 0.20 m | 暫定hazard=1となるobstacle heightの目安 |
| `publish_hazard_cause_markers` | false | 最大寄与cueをRVizで色分けしたcubeとして出力。Jetson負荷を避け既定OFF |
| `hazard_marker_max_points` | 2500 | 色分けmarkerの最大cube数。超過時は均等間引き |
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

Stage 3.1では、局所平面残差roughness、heading support付きstep、最大寄与cueの
色分けdebugまでを実装済みです。plane fitは最小二乗でありvegetation/外れ値にはまだ
堅牢ではなく、hazard閾値も安全な走行可否を決めるものではありません。次は
(1)ground候補へのrobust estimator、(2)vertical bandsまたはnegative-obstacle cue、
(3)明示的な軽量terrain map message、(4)検証済みのcost変換、の順が妥当です。Nav2
pluginはその後にします。
