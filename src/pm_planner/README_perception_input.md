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
以前の危険領域の再観測による解除はproducer側の責務。低い観測や時間経過だけで解除しない。
車体衝突領域をどこまで観測できたかもproducer側で評価する。枝の下を通れる等の判断を
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
衝突しない候補がなくなった場合は停止し、入力回復だけでは再開しない。

## 時刻・座標・負荷

stamp・受信時刻をそれぞれterrain_timeout_sec（既定1秒）で検査する。
モード変更前のmap、重複・逆順のsnapshotで鮮度を更新しない。
異なるframeはmap stampのTFで変換する。latest TFを代用しない。

新headerでもセルの根拠は古い場合がある。セルの根拠の有効性・観測可能性・解除をproducerが
判断してcost/unknownへ反映する。plannerの1秒は配送の健全性であり、セル保持のTTLではない。
静止中の地面を時間だけで消す規則を追加したわけではない。

producerは物体と観測誤差の余裕、plannerは車体footprintを担当し、車体の二重膨張を避ける。
円形footprint半径0.4 mは実測確認が必要。膨張は受信時、候補内はNumPy配列参照を使う。
新しい2channelは10 cm/8 m mapなら合計12.8 kB/snapshot程度で、全pixelや点群は追加しない。

## 既存perceptionとの接続状況

**本変更では並行開発中のpm_perception本体・認識規則を変更していない。**
既存の通常mapperはまだTerrainAssessmentを発行しないため、perception側が上の契約で
専用publisherを追加するまで、認識ONの走行準備は成立しない。OFFの経路追従は利用できる。
受信・選択・MPPI・GUI・停止までの接続は、専用messageを発行する隔離試験で確認する。

既存terrain_hazard_debugは、obstacle unknownを他のcueの0が隠す場合がある。
obstacle_height_debugも観測済み非検出を0として表現しない。
したがって、どちらも自動fallbackには使用しない。
`allow_legacy_full_hazard: true`は旧総合debug mapの比較試験だけの明示opt-in。
full_terrain以外には使わず、通常運用の代替とはしない。

producer追加時の最小例（値は認識ロジックの出力から構築し、架空のfreeを作らない）：

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
