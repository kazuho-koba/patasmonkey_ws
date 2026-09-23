User prompt:
このセッションの名前は「ローカルプランニング・traversability estimation」としてください
patasmonkey_ws に、OAK-D S2 の depth image を利用したローカル地形認識機能を段階的に追加したいです。

最終的には、UGV前方のdepth観測を撮像時刻の自己位置・姿勢に基づいて odom 座標系へ融合し、robot-centric な rolling 2.5D terrain map を生成して、Nav2のローカルプランニング用costへ変換することを想定しています。

ただし、一度に最終形まで実装しないでください。今回は、既存コード・topic・TF構成を確認した上で、後から段階的に拡張しやすい最小構成を設計・実装してください。

## 重要な前提

* ROS 2 Foxy
* 実機は Jetson AGX Xavier
* OAK-D S2 を使用
* カメラ高さはおよそ 0.4 m
* 車幅はおよそ 0.45 m
* 屋外・オフロードUGV
* 計算リソースは潤沢ではないため、CPU負荷・メモリコピー・ROS message生成量を抑えることを重視する(たとえば、PointCloud2を必須の内部表現とはせず、depth image から直接2.5D gridへ投影する方式等も考慮する)
* 過去のrosbagを使ってオフライン検証することが当面の主な試験方法
* bagの再生そのものや可視化ツールの実行そのものをあなたが行う必要はない。ただし、それらを人間が容易に実行できるスクリプト・launch・README等は作成してほしい
* localization側の既存実装を不用意に変更しない
* legacy/current behaviorを壊さない

## 最終的に目指している構成

概念的には以下です。

OAK-D depth image
→ depth pixel の間引き・品質チェック
→ camera intrinsics で各pixelを3Dへback-project
→ depth画像のtimestampに対応する TF を使って camera frame → odom frame へ変換
→ odom座標系の rolling 2.5D grid へ直接融合
→ elevation / variance / obstacle情報等を保持
→ slope / roughness / step / traversability を後段で計算
→ 最終的にNav2 costmapへ入力

ただし、今回は slope / roughness / traversability / Nav2 plugin まで実装する必要はありません。

## 今回まずやってほしいこと

### 1. 現状調査

まず patasmonkey_ws と関連する ros2_ws を確認し、以下を調査してください。

* OAK-D S2 のdepth image topic
* camera_info topic
* depth画像のencoding、解像度、想定Hz
* camera optical frame名
* base_link等へのstatic TF
* odom → base_link のTFまたはOdometryの供給元
* そのTFが x/y/z/roll/pitch/yaw のうち何を実際に推定しているか
* depth timestampでTF lookupできる構成になっているか
* rosbagに必要なtopicとTFが記録されているか
* pm_config / pm_bringup 等の既存構成との整合性
* 新規packageをどこへ置くのが自然か

特に、terrain mappingに使える6DoF情報の実態を確認してください。

調査結果を簡潔に整理してから実装方針を決めてください。

### 2. 新規 pm_perception package の設計

新規に `pm_perception` packageを作る方向で検討してください。

最初から大きなmonolithic nodeにせず、将来的に以下を分離・交換できる構造にしてください。

* depth input / sampling
* depth → 3D back projection
* timestamped TF transformation
* rolling grid management
* terrain feature calculation
* visualization / debug output
* Nav2 interface

ただし、現段階では不要な抽象化や複雑なplugin architectureは作らないでください。

Jetson AGX Xavier上で軽量に動かすことを優先してください。

### 3. Stage 1 の最小実装

今回はまず、depth imageから直接2.5D elevation gridを構築できるところまでを実装してください。

初期値の候補は以下です。既存環境を確認して合理的な変更があれば提案してください。

* rolling map: 6 m × 6 m程度
* resolution: 0.05 m
* map frame: odom
* depth使用範囲: およそ0.4～4 m
* depth processing rate: 最大10～15 Hz程度から開始
* depth pixel sampling: まず4×4程度の間引きを候補とする
* full resolution PointCloud2は通常生成しない
* map外へ出たセルはrolling mapとして再利用
* depth画像取得時刻のTFを必ず使用する
* latest TFを無条件に使用しない

最初のgrid layerは必要最低限で構いません。

少なくとも候補として、

* elevation
* elevation variance または簡易confidence
* observation count
* last observed time

を検討してください。

ただし、「1セルにつき単純に最後に観測したzを上書き」のような方法ではなく、複数観測を軽量に融合できるようにしてください。

一方で、最初から高度なBayesian elevation mappingを実装する必要はありません。

### 4. 地面以外の点を将来扱える構造にする

今後、

* 木
* 人
* 岩
* 乗り越え不能な段差
* vegetation
* negative obstacle

等も扱いたいです。

そのため、将来的に各XYセルに

* ground elevation
* obstacle height
* obstacle confidence
* last obstacle observed time
* vertical occupancy bands

等を追加できるデータ構造にしてください。

ただし、今回それらを全部完成させる必要はありません。

「地面」と「地面より上の物体」を分離するアルゴリズムについては、今回は最低限の実装またはstubでも構いません。

### 5. 可視化・オフライン評価手段

過去のrosbagを再生して、人間が結果を確認できるようにしてください。

最低限、以下のいずれか、可能なら複数を用意してください。

* RViz2でelevation mapを確認できるtopic
* OccupancyGrid等へのdebug変換
* elevation mapを画像として保存・表示するPythonスクリプト
* 時系列でrolling mapがどのようにつながっていくか確認できる可視化
* 必要ならdebug用PointCloud2。ただし常時生成はしない
* bag再生時に必要なlaunch/config例

特に、
「2 m程度前方で観測した地面が、車両が接近して現在のcamera FOV外になった後も、odom上の同じ場所としてmapに残っている」
ことを確認しやすくしてください。

可視化用処理によって実機runtime側の負荷が増えないよう、debug出力はparameterで無効化できるようにしてください。

### 6. 計算量を意識した実装

Jetson AGX Xavierでの使用を前提として、計算量の削減を常に考慮してください。
必要なら、1 frame当たりの処理時間を計測する簡単なdiagnostic/logも入れてください。

### 7. 今回やらないこと

今回の最初の実装では以下は不要です。

* 完成版traversability推定
* semantic segmentation
* 人認識
* full 3D voxel mapping
* GPU/CUDA最適化
* Nav2 custom costmap plugin
* autonomous plannerとの統合
* localization側の大規模変更

これらは後続Stageで追加します。

## 実装時の進め方

一度に大量の変更をせず、

1. 現状調査
2. package/API/topic設計
3. 最小実装
4. build確認
5. bagで検証するための可視化・実行方法整備
6. 次StageへのTODO整理

の順で進めてください。

既存コードや設定から判断できない点については、勝手に大きな仮定を置いて既存システムを書き換えず、parameter化またはTODOとして残してください。

実装後には以下を報告してください。

* 追加・変更したファイル
* node/topic/TFの構成
* depth→gridの処理フロー
* 各主要parameterと既定値
* Jetson上の計算負荷を抑えるために行った工夫
* rosbagを使った確認手順
* 現段階で分かっている制約
* 次に実装すべきStage 2の候補

まずは既存workspaceとTF/topic構成の調査から開始してください。

実装について基本的にはpythonでお願いできないでしょうか。ここまで進めた分についてもc++のものがあれば変えてください。
c++でないととても速度が出なさそうなときには、スクリプトに厚めにコメントをつけるという条件で許可します。

Codex final response:
Stage 1のPython実装まで完了しました。`pm_perception`はC++骨格を削除して、NumPyでベクトル化した`ament_python` packageへ置き換えています。

主な調査結果：

- depthは`/oak/depth/image_raw`、`16UC1`（mm）、640×400、設定10 Hz・bag実測約10.01 Hzです。
- 過去bagには`/tf`、`/tf_static`、`/odometry/local`、depthが記録されていますが、CameraInfoはありませんでした。
- depth headerの`oak_rgb_camera_optical_frame`とURDFの`rgb_camera_optical_frame`が不一致でした。旧bagはframe override、新規driverはURDF名へ統一しています。
- static TFは`base_link → oakd_link → rgb_camera_link → rgb_camera_optical_frame`です。ただしURDFのカメラ高さ0.28 mは、提示値約0.4 mと不一致で要実測です。
- legacy構成はrobot_localizationが6DoF `odom → base_link`をpublishしますが、zはVIO速度積分のためdriftし得ます。separated_offroad構成はx/y/yawをhorizontal EKF、roll/pitchとgate済みVIO zをobserver/composerが供給します。代表bagではdepth期間全体を50 HzのTFが覆っていました。

実装内容：

- [pm_perception README](/home/kazuho/Projects/patasmonkey_ws/src/pm_perception/README.md)に調査結果、設計、全parameter、build・bag再生・RViz手順、制約を記載しました。
- `depth_projection.py`で4×4 sampling、depth品質チェック、pinhole back-project、NumPy一括座標変換を行います。
- `rolling_elevation_grid.py`は6×6 m、0.05 m（120×120）の固定長ring bufferです。全mapを移動コピーせず、odom絶対cell tagで領域外slotを再利用します。
- groundはframe内のセル最小zをWelford逐次平均・分散へ融合し、`elevation`、分散、観測数、最終観測時刻を保持します。将来用にobstacle height/confidence/時刻も確保しました。
- `depth_elevation_mapper_node`はdepth撮像時刻でcamera→odomとbase_link→odomをlookupします。TF未到着frameはbounded queueで待ち、timeout後にdropし、latest TFにはfallbackしません。
- debug出力は`~/elevation_debug`（OccupancyGrid）と、既定OFFの`~/elevation_points_debug`（1点/cell PointCloud2）です。
- `depthai_driver`へEEPROM由来の`/oak/depth/camera_info` publishを追加し、3つのbag launchへ記録topicを追加しました。既存localizationの処理・interfaceは変更していません。

主要既定値はdepth 0.4–4.0 m、stride 4、処理上限12 Hz、TF待ち0.25 s、ground融合閾値0.20 m、観測分散floor 0.0025 m²、debug publish 2 Hzです。

検証結果：

- Foxy container build: `depthai_driver`、`pm_perception`とも成功
- unit test: 4/4成功
- `ament_flake8`: 問題なし
- 旧bagスモークテスト: timestamped TF drop 0、平均約2.2 ms/frame、最大約6.1 ms/frame。4×4 sampling後は最大約15,300点/frameでした。
- `git diff --check`: 両workspaceとも問題なし

旧bag確認はREADME記載どおり、`terrain_mapping_old_bag.launch.py`、`ros2 bag play ... -s mcap --topics /oak/depth/image_raw /tf /tf_static`、同梱RViz設定の順で実行できます。Fixed Frame=`odom`で、FOV外になった既観測地点がrolling window内に残ることをPointCloud表示で追跡できます。

現段階の主な制約は、カメラ取付外部パラメータが暫定、旧bag fallback intrinsicsが特定OAK個体用、最小z ground仮説ではvegetation/negative obstacleを分類できない、ROS出力はまだdebug APIのみ、という点です。Stage 2は、外部パラメータ確定、距離依存分散とaging、ground/obstacle二仮説・vertical bands、slope/roughness/step、軽量map messageの順を推奨します。
