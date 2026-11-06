# perceptionとplannerの専用入力・認識モード

## モードとGUI

走行開始／終了と認識モードは独立する。Operator Consoleの認識modeを選んで適用し、
UGVの実行管理が受け付けたmodeを表示で確認する。切替時は停止するため、走行開始を明示操作する。

| mode | MPPIが使う環境情報 |
|---|---|
| off | 使わない。従来のGNSS経路追従は可能。perceptionの処理自体は止めない |
| obstacle_only | 独立したobstacle_costだけ。他の3指標が危険でもこのmodeでは混ぜない |
| full_terrain | obstacle_costとterrain_costの両方。路面ON・障害物OFFという組合せは作らない |

サービスは`/pm/planner/execution_manager/set_perception_mode`、型は`pm_msgs/srv/SetPlannerMode`。
request.modeは上記の3文字列、responseはsuccess・実際のmode・message。
同じmodeの再要求は停止させない。mode変更は停止・車両許可解除・旧map無効化を行い、
変更後の新しいmapと明示開始を必要とする。OFFも自動走行の再開指令ではない。
手動介入、GUI heartbeat、自己位置、車両状態、mission/Pathの版の検査は従来どおり。
認識modeは自律MPPIの設定であり、手動Joy指令への障害物回避フィルタではない。

GUIはmode適用のサービス応答を待ち、UGVのstatusから実modeを表示する。
画面で選んだだけの値を実際に有効なmodeとは扱わない。
旧UGVが新サービスに未対応の場合、GUIの適用ボタンは無効になる。

## producerとの契約

専用topic：`/depth_elevation_mapper/planner_costmap`
型：`pm_msgs/msg/TerrainAssessment`

| field | 契約 |
|---|---|
| schema_version | 1。未知のversionを受け入れない |
| header | frameと評価snapshotのstamp。内部セルの観測時刻とは区別する |
| info | nav_msgs/MapMetaData。共通の解像度、原点、width/height |
| obstacle_cost | row-majorのint8配列。width×height要素が必須 |
| terrain_cost | 同じgridのslope/roughness/step由来のcost。obstacleを混ぜない。未提供は空配列 |

共通の値：

- -1：評価不能、未観測、遮蔽、必要な基準やsupportの不足など。
- 0：その評価項目について必要な領域を有効に観測し、定義した危険条件で非検出。
- 1〜99：非致命の評価cost。
- 100：危険。衝突を否定できない高点の確認待ち・確認済みはどちらも危険として扱う。

0は安全認証ではない。支持不足・欠測・低い再観測・期限切れを一括で0に変えてはならない。
以前の危険領域の解除はproducer側の責務。Coreの既定producerは
`pm_perception/planner_mapper_node`で、N画像窓の有効高さ平均による解除を行う。
欠測窓による記録解除はunknownへ戻し、安全値0へ変換しない。背景rayは要求しない。
この規則は物体消失の証明ではなく、遮蔽・視野外でもunknownへ戻り得る。
詳細は`pm_perception/README_planner_perception.md`を参照する。
枝の下を通れる等の判断を
単なる高低差やカメラ対地高さから推定しない。

terrain_costは、必要な三指標が有効ならその正規化最大値を使う。未評価があれば-1、
ただし一つの指標だけで既知の危険があれば100とする設計を想定する。
閾値の選択・校正、road基準、確認・解除規則はperception側で定義する。
plannerは観測点から再認識したり、既存の暫定閾値を本番安全限界と認定したりしない。

同じmessageへ独立channelを入れるので、別topicの時刻ずれやrolling原点ずれを避けられる。
最大要因のcauseラベルからobstacle_costを復元しない。例えばterrain=100・obstacle=70でも
obstacle_onlyでは70を保ち、fullでは100を使用する。

## mode毎のunknownと危険

obstacle_onlyはobstacle_costのみを使い、terrain_cost未提供でも利用できる。
full_terrainは両方が既知なら最大を使う。一方がunknownならunknownを保持する。
一方だけで危険閾値以上と分かる場合は、他方がunknownでも100として停止に使う。

```text
obstacle=-1, terrain=0   → full=-1（obstacle未知を総合の白で隠さない）
obstacle=0,  terrain=100 → obstacle_only=0 / full=100
obstacle=70, terrain=100 → obstacle_only=70 / full=100
obstacle=100,terrain=-1  → full=100（既知の危険をunknownで隠さない）
```

unknown・map外・危険閾値以上は候補不採用。それ以下は従来のterrain_cost_weightによるcost。
車体footprint膨張、step間検査、重み付き平均後の再検査を共通に使う。
現在のfootprint自体が危険・unknownなら開始を拒否する。
`keep_autonomy_on_blocked: true`（既定）では、衝突しない候補がない場合や現在footprintが
危険・unknownになった場合、`WAITING`へ移りゼロ指令を継続し、自律許可を維持する。
入力が健全なまま停止速度・yaw rateの確認ができ、有効な候補が得られれば自動続行する。
進捗は毎周期監視し、待機前の速度指令と最適解は破棄してゼロから再計画する。
待機に自動終了期限は設けていない。

falseでは従来どおり無効化・許可解除となり、GUIから明示開始が必要。
どちらの設定でも、自己位置異常、mapの欠落・不正・期限切れ、GUI切断・停止、手動介入、
車両許可解除、緊急停止、経路更新、制御周期超過は許可解除する。入力が戻っただけでは再開しない。
初回開始時のfootprintが危険・unknownなら、開始拒否の条件は維持する。
`enabled=true`かつ`state=WAITING`は許可を維持したゼロ指令状態であり、走行中を意味しない。
statusのwaiting_for_obstacleとwaiting_sec（wall clock秒）でも確認できる。

## 時刻・座標・負荷

stamp・受信時刻をそれぞれterrain_timeout_sec（既定1秒）で検査する。
モード変更前のmap、重複・逆順のsnapshotで鮮度を更新しない。
異なるframeはmap stampのTFで変換する。latest TFを代用しない。

新headerでもセルの根拠は古い場合がある。セルの根拠の有効性・観測可能性・解除をproducerが
判断してcost/unknownへ反映する。plannerの1秒は配送の健全性であり、セル保持のTTLではない。
静止中の地面を時間だけで消す規則を追加したわけではない。

producerは物体と観測誤差の余裕、plannerは車体footprintを担当し、車体の二重膨張を避ける。
通常plannerの現在設定はfootprint_radius_m=0.3 m、footprint_margin_m=0.1 m。
膨張半径は両者の和0.4 mで、追加余裕はm単位で解像度から独立する。margin=0も指定可能。
セル幅を変えても設定した余裕は変わらないが、セル格子による離散化誤差は残る。
車体寸法・位置誤差を含む実測確認が必要。膨張は受信時、候補内はNumPy配列参照を使う。
追加余裕は障害物・unknown・map端の全てへ同じように適用する。

通常起動はpm_config/config/planner.yaml、単独はpm_planner/config/planner.yamlを編集し再起動する。
非駆動PerceptionPreviewも共有設定を既定値として読み、同名parameterを環境生成へ渡す。
probeはmarginをsnapshotへ保存し再現する。marginのない旧snapshotと旧Python API呼出しは、
従来の1セル分を追加する挙動を互換維持する。新nodeは常にm単位の値を明示する。
新しい2channelは10 cm/8 m mapなら合計12.8 kB/snapshot程度で、全pixelや点群は追加しない。

## 既存perceptionとの接続状況

現在の`pm_perception/planner_mapper_node`がTerrainAssessmentを発行する。
標準Coreはこのproducerを起動し、共有`pm_config/config/planner_perception.yaml`を使う。
通常のGUIで「障害物回避ON」を適用し、mission・自己位置・mapが準備できてから
走行開始を操作する。走行許可と中立確認は従来どおり別途必要。

`planner.launch.py`単独はproducer・localization・車両interfaceを起動しない。
Coreと重複起動せず、それらを別途用意する。比較用の旧`depth_elevation_mapper_node`は
専用出力を発行しないため、認識ONで使うproducerと混同しない。
認識の判定・解除規則は[producer README](../pm_perception/README_planner_perception.md)参照。

plannerのnamespace下でもTFは共通の`/tf`・`/tf_static`を購読する。
Foxyの相対TF topicが`/pm/planner/tf`へ解決される問題を修正し、実producerのodom gridを
mapのPathへ変換する結合試験で確認した。最新TFへの代用は行わない。

実producerのモデル・publisher、GUI backend、MPPI、実vehicle_interface_nodeを用いる
非駆動結合試験が`test_producer_vehicle_pipeline.py`。
投影後の人工点群・人工自己位置を入力し、ODrive依存はimport前にMemoryMotorへ置換する。
GUIからのmode・開始要求、非ゼロcmd_vel_autonomyの採用とモータ指令変換、
車両の0.3秒指令期限、危険map・認識途絶による停止、Joyでの許可解除を検証する。
これは実機走行やデプス撮像からの全段検証ではない。

既存terrain_hazard_debugは、obstacle unknownを他のcueの0が隠す場合がある。
obstacle_height_debugも観測済み非検出を0として表現しない。
したがって、どちらも自動fallbackには使用しない。
`allow_legacy_full_hazard: true`は旧総合debug mapの比較試験だけの明示opt-in。
full_terrain以外には使わず、通常運用の代替とはしない。

別producer実装時の最小例（値は認識ロジックの出力から構築し、架空のfreeを作らない）：

```python
from pm_msgs.msg import TerrainAssessment
# node初期化時
publisher = node.create_publisher(TerrainAssessment, '~/planner_costmap', 1)
# 同一評価snapshotを作成した時
message = TerrainAssessment()
message.schema_version = 1
message.header = assessment_header
message.info = common_grid_info
message.obstacle_cost = obstacle_cost.astype('int8').ravel().tolist()
# 三指標が未実装なら[]。空を0で埋めない。
message.terrain_cost = terrain_cost.astype('int8').ravel().tolist()
publisher.publish(message)
```

producerの本番評価規則と、初回検出・誤解除・coverage・車体寸法・遅延・負荷は別途検証が必要。
大きな迂回を見つけるグローバル再探索も本機能には含まない。

## ビルドと確認

開発PCはFoxyコンテナで、Foxy→外部overlayをsourceしてからビルドする。

```bash
colcon build --symlink-install --packages-select pm_msgs pm_mission pm_config pm_planner pm_gui pm_bringup
```

実行時はmain overlayもsourceする。UGV・GUI双方へ新しいpm_msgsをデプロイする。
通常のCore・Operator Consoleを使用し、認識modeを適用後に走行開始を操作する。
現在のmode・input準備状態・拒否理由はexecution_manager/statusにも出る。
新topicはmission bagへ追加し、statusと同時に記録して選択modeと指令を検証できる。

## 希望離隔と接近時の低速化

必須の禁止範囲は従来どおり`footprint_radius_m + footprint_margin_m`で決める。
その外側の通行可能な場所には、次の希望余裕を設定できる。

```yaml
clearance_distance_m: 0.6
clearance_cost_weight: 3.0
clearance_min_speed_mps: 0.2
```

`clearance_distance_m`は**必須膨張の外側から追加で確保したい距離[m]**。
半径0.3 m・必須余裕0.1 mなら、障害物セルから中心まで概ね1.0 m以上で
離隔costがゼロになる。セル離散化のため厳密な連続面の距離ではない。
障害物・unknown・map外を同じ対象として扱う。

膨張済み禁止領域までの距離をd、希望追加離隔をDとして、
不足率r=clip(1-d/D,0,1)、追加cost=clearance_cost_weight×r²を各予測stepに加える。
広い場所は離れて回避し、狭い場所ではcostを受け入れて必須余裕を守って通れる。
OpenCVの距離変換でセル中心間距離を求め、1セル分を差し引いて保守的にする。

速度上限はtarget_speedからmin(target_speed,clearance_min_speed_mps)まで不足率で線形に下げる。
これは最低走行速度ではなく、旋回・終点・待機は0.2 m/sより低くできる。
既に高速で近づいている場合は、既存の加減速度制約を優先して徐々に減速する。
距離不足に対する低速化であり、実機応答遅れを含めた停止距離の保証ではない。
必須領域への侵入候補は従来どおり不採用、有効候補なしなら停止待機となる。

D=0で新しい離隔cost・減速を無効化できる。重み=0はcostだけを無効化し、減速は残す。
perception OFFには適用しない。設定は通常起動用pm_configと単独用pm_plannerのYAMLにあり、
通常plannerは起動時に読み込むため変更後は再起動する。
非駆動perception_previewにも同じ設定を渡し、probeはMppiConfigとともに保存する。
旧probeに離隔設定がなければ、当時の比較を維持するため無効として再現する。

Jetson向けに、距離変換はmap更新時だけ行い、MPPIではNumPy配列参照だけを使う。
通常plannerは構築したmapを使い回し、previewも同じmapと設定なら再構築しない。
PointCloudや新しいROS topicは生成しない。実行依存はpython3-opencv。
