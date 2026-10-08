# 独立frameの高さ範囲付き物体候補診断

`tools/diagnose_height_candidates.py`はオフライン専用のPythonスクリプト。
通常`pm_perception`のメインnode・地形計算・ROS interfaceを変更しない。
既存bagと通常mapperの実使用TF traceを読み、毎frame空の地形gridを作る。

## 今回の実装範囲と、まだ実装していないもの

| 機能 | 今回 |
|---|---|
| 独立frame内のセル・高さgap集約 | 実装済み。壁・枝を必ず平面へfitしない |
| groundとは別の高さ範囲付き候補 | 実装済み。1点群も保持し、縦幅を平均値へ潰さない |
| 比較可能な数観測の高さ移動平均 | 別scriptで直近3対応観測の診断平均。実機には未導入 |
| 再観測による存在支持の増加／否定 | 別scriptで3回positive支持を集計。不在の否定証拠・確率校正は未実装 |
| confidence画像との対応 | 未実装。DepthAIのpixel confidenceと存在支持は別概念 |
| rayによるfree確認・過去障害物の解除 | 未実装。高点なし、細い幅、候補数制限でsafeにしない |
| 候補を用いた完成版hazard・footprint通過評価 | 完成版は未実装。別scriptで通過前footprintの診断対照あり |

議論した短期3観測平均の前に、frame内候補の表現が本物の壁・細枝を消さないか
を検証する第一段階。ground＋高点を一緒に平均する処理は入れていない。
今回の出力だけで「誤黒率が下がった」「安全性が向上した」と判断しない。

## 処理とgroundの注意

1. traceに記録された画像stampだけをbagから選び、同stampのcamera→odom TFとKを使う。
2. traceのstride・depth範囲で、現行の`sampled_points`を使ってback-projectする。
   stride>1では代表画素が無効の区画内を再探索する。traceは画素リストを持たないため、
   古いsampling実装の画素列まで厳密に再現したという意味ではない。
3. 既存`evaluate_points`で独立frameの従来cell最小z・平面・hazardを作る。
4. 現行featureの非退化平面でslope／roughnessが既存閾値未満なら診断基準へ使う。
5. 高さgapで候補を作り、z下端／上端、平均、点数、XY範囲、元の極値画素を保存する。

最下群が薄く、全点が平面の整合幅内なら`ground_provisional`。
**これはgroundの確定ではない**。平らな天井・岩・誤った平面も通りうる。
groundが未確定でも高点・縦幅・疎な群は未分類候補として残す。
ground成立を仮定した残差hitには`above_plane_hit`という名前を付ける。

`body_height_hit`は、有効な暫定平面からの高さが
`hazard_obstacle_height_limit <= residual <= vehicle_height_m + safety_margin_m`
に入る**実測点**の有無。区間内部を補完しない。車両全高nullならunknown。
これは各XYセルの高さ域診断であり、姿勢付き車体の掃引体積との完全な衝突判定ではない。
低い物体の乗り越え限界や車両姿勢は別途必要。レンズ対地高さ0.4mを全高へ代用しない。

候補は非ground／未分類で最大2枠、ground仮説は別枠。gapで分裂した疎な壁などで
枠を超えればoverflowとして数え、全処理対象点に基づくhit情報は枠制限前に集約する。
少数候補の高さ範囲は外包であり、groundと天井の間をoccupied／freeへ補完しない。

## 実行

Foxyコンテナ内でROS、外部workspace、main workspaceの順にsourceする。
新規Python moduleを使うため、最初は`pm_evaluation`を通常手順でbuildする。
スクリプトはsource treeから実行する。ROS node・bag再生・RViz起動は不要。

```bash
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
source /workspaces/patasmonkey_ws/install/setup.bash
cd /workspaces/patasmonkey_ws
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1

python3 src/pm_evaluation/tools/diagnose_height_candidates.py \
  bags/rosbag2_2026_07_26-09_17_38 \
  --mapper-trace notes/reports/perception/20261003_100000_depth融合_stride比較_assets/stride4/trace/mapper_trace.jsonl \
  --config src/pm_evaluation/config/height_candidate_diagnosis.yaml \
  --output /tmp/height_candidates_sample \
  --every-seconds 15 --max-frames 24 --save-frames 3
```

全採用frameなら`--every-seconds 0 --max-frames 0`。出力先は新規directoryのみ。
正の間隔は動作確認用で、採用画像全体や通過経路の統計とは区別する。
MCAP専用。別bagはそのbagを元に採ったmapper traceを指定する。
trace内画像のstamp・解像度・frame名は検査し、欠落を別stampやlatest TFで補わない。
stamp等が偶然一致する別bagまで識別できる保証はないため、bagとtraceの対応は必須。
QoSはファイル読取には関係しない。trace生成時のRELIABLE／history等はsummaryへ残る。

`config/height_candidate_diagnosis.yaml`で候補gap・薄い幅・整合幅・候補数・車両全高を
変更可能。sampling・grid・hazard閾値はtraceのmetadataから読み、勝手に現行YAMLへ
置換しない。候補閾値は試行値で、センサ誤差モデルにより校正された値ではない。

## 出力と同条件比較

- `candidates.csv`：stamp、odomセル、候補種別、z範囲・平均、点数、XY範囲、距離、
  極値画素u/v、平面残差、暫定ground基準の可否。存在確率null、free証拠false。
- `frames.json`：同じ点列の従来raw幅黒セル数・reference hazardの黒／既知／unknown、
  新候補数・疎な群・広い群・overflow、暫定平面hit。分母はframe×mapセルであり、
  footprintが黒セルを通過した割合ではない。
- `cells.csv`：観測セルごとの全候補数・overflow・hitと、根拠画素u/v・odom XYZ。
  候補枠外のhitも保存する。平面無効／全高未指定は空欄で、0（否定）と区別する。
- `summary.json`：入力・trace hash・parameter・対象枚数、処理時間mean/p95/max、RSS。
  source hashも保存して、現在のsampling／feature実装を特定する。
- 選択frameのNPZ／PNG：原点・従来gridと候補数・縦幅／overflow診断を並べる。

**従来の高さ幅hazardと新しい候補のhitは同じ意味ではない。** ground／plane基準が
unknownなら新hitもunknown。unknownを0へ置換して黒率減少と呼ばない。
時間融合mapperとの過去の通過割合へ、今回のframe内件数を直接比較しない。
まず同じframe・点列のraw min/maxと候補極値、危険点保持、分類未確定を確認する。

## 軽量化とテスト

全点をcell・z順に一度sortし、観測セルだけ処理する。全セルGMM、full voxel、
raycast、PointCloud2生成・配信はしない。CSVはframe単位にstream、図／NPZは指定枚数のみ。
純NumPy coreとI/Oを分離し、診断でPython objectを使う部分を実機runtimeへ持ち込まない。
RSSはMCAP reader・従来評価・matplotlibを含むプロセスpeakで、候補だけの増加量ではない。
処理時間は読取／描画を除くframe計算と候補処理を分け、Jetsonの達成Hzは別試験とする。

```bash
python3 -m pytest -q src/pm_evaluation/test/test_height_candidates.py
```

ground＋単発高点、縦に連続した壁、疎な枝、天井との隙間、overflow、空／無効点を
合成で確認する。原画像・RGBによる物体正解の照合は、これとは別の手動診断が必要。

## 追加診断：平面棄却理由と原画像

`cells.csv`には`plane_reason_bits`と個別bool、support数・平面係数・slope・roughness
を保存する。bit 1=self unknown、2=support不足、4=配置退化、8=slope棄却、16=roughness
棄却。重複はbit和。これは既存gateの説明であり、groundアルゴリズムの改善ではない。

```bash
python3 src/pm_evaluation/tools/plot_height_candidate_evidence.py /tmp/height_candidates_sample \
  --bag bags/rosbag2_2026_07_26-09_17_38 \
  --mapper-trace notes/reports/perception/20261003_100000_depth融合_stride比較_assets/stride4/trace/mapper_trace.jsonl \
  --output /tmp/height_candidate_evidence
```

理由別最大高さ幅、overflow、高点hitの代表例をdepth元画素・XYZ・最近接RGBで図化する。
RGBは環境文脈でありdepthとの画素alignは行わない。最大例は典型頻度を代表しない。

## 追加診断：連続frameのN=1／N=3比較

```bash
python3 src/pm_evaluation/tools/diagnose_height_candidate_sequence.py \
  bags/rosbag2_2026_07_26-09_17_38 \
  --mapper-trace notes/reports/perception/20261003_100000_depth融合_stride比較_assets/stride4/trace/mapper_trace.jsonl \
  --config src/pm_evaluation/config/height_candidate_diagnosis.yaml \
  --z-match-gate 0.05 --xy-match-gate 0.03 \
  --output /tmp/height_candidate_sequence
```

全採用frame・同じTFを使用。odom同一セル内で高さ区間間距離・XY外包間距離をgateし、
平均z差の小さい順に一対一対応する。ground仮説とobjectは対応させない。これは
物体の同一性を保証しない。broadとthin／sparseを混ぜず、broad同士は高さ区間の
overlap/union>=0.5、その他は平均z差もz gate以内を要求する。broadの代表高さ差は
残りうるため、壁を高さ平均の一面として使わない。
各trackの直近3対応観測で高さ平均と区間外包を保存する。broadの平均を物体面にしない。
窓は秒数ではないが、停止時の連続画像も数えるので独立した3証拠とは限らない。

`tracks.csv`に対応、単frame高さ、N=3平均、幅、positive支持、pending／confirmedを
stream保存する。存在確率は未推定。非観測・遮蔽・unknownは中立で、free-ray検証と
解除は未実装。track在庫は診断用に無制限なので実機runtimeへそのまま使わない。

`footprints.csv`／`summary.json`は0.25m移動ごとの0.45×0.55m近似footprintについて、
**通過stampより前のframeだけ**から比較する。観測ROIはframeのrolling map範囲で、
以前の前方半径2／5m・近距離優先A評価とは異なる新しい対照実験である。

- `raw_width`：従来セル内高さ幅を保持する対照。
- `N1`：有効な暫定平面からの実測high-hitを初回から保持。
- `N3_confirmed`：直近3対応観測すべてがhigh-hitになったものだけ。**安全出力ではない**。
- `N3_conservative`：確認待ちを含め初回high-hitを保持。危険を平均で消さない。

terrain三指標は同frameで3つ揃った組を保持し、平均高さから再fitしない。全cue既知、
いずれか既知、全unknownは別に数える。危険値は負のfree証拠なしに解除しないため、
N1とN3保守側は同じ出力となる。単純な3回確認の黒率低下を誤検出改善と呼ばない。
確認までの時間／移動距離は同track初回positiveから計算し、同セルの別trackを含む
初回hit→初回確認時間とも区別する。実障害物の正解なしに検出漏れは判定できない。

## 今後必要な検証

### 保存された通過評価の黒原因・更新状態を確認する

連続frame診断は`footprints.csv`に4cue別の黒セル数、`black_cells.csv`に黒の指標値・
出典stamp・最後の観測状態を保存する。ground gate不成立はunknownであり、直接blackには
しないが、過去のblackが保持される場合を調べられる。最新画像で未観測だった状態と、
そのセルの最後の実観測で平面基準が不成立だった状態は区別する。

```bash
python3 src/pm_evaluation/tools/analyze_height_footprint_causes.py \
  /tmp/height_candidate_sequence_with_causes \
  --reference notes/reports/perception/20261006_231622_高さ候補のground診断と短期窓比較_assets/sequence
```

入力は上の連続診断を新規出力先で実行したdirectory。旧結果の全既存footprint列と
summaryの割合が一致しなければ停止する。`cause_audit.json`に重複cue・排他的組合せ・
状態別件数を出す。N=3確認済みのobstacle出典stampは初回確認、N=1は初回positive。
最後のhigh-hitなしだけでsafeへ解除できるとは解釈しない。TF／DDS失敗等の実機障害を
このオフライン状態診断から推定することもできない。

候補の対応付けが成立することを確認してから別parameterで追加する。

1. 同じセル・高さ・XY領域・観測品質の候補を対応付ける。姿勢／ground基準ずれを診断。
2. **薄い対応候補のヒット高さだけ**を直近3つの比較可能観測で平均／中央値比較する。
   broadな壁の上下端を単一の高さ平均へ潰さない。
3. 高さ値と存在支持を分ける。高点がないframeを高さ0として混ぜない。
4. 高点へのhitは支持、適切なray／画像領域で背景まで見えた場合だけ否定証拠。
   欠測・遮蔽・sampling漏れはneutral。停止による時間消去は入れない。
5. N=1／3の同条件比較で、確認・解除までの回数／移動距離、危険候補の見逃し、
   footprint黒／unknown率、CPUを評価する。3反復で実物と確定したとはしない。

`z_mean_m`は**同一frame内・同一候補の平均**、track出力`mean_z_m`は直近対応観測の平均。
どちらも平面fitの高さではない。ground gate、free証拠、対応安定性の改善は未完成。

## N=5比較と確認待ちの観測数

連続診断のコマンドに`--window 5`を追加し、別の新規`--output`へ保存する。
既定は3で、N5では`N5_confirmed`／`N5_conservative`が出る。通常mapperは変更しない。
Nは画像数／秒数ではなく対応候補観測の数。5全てpositiveなら確認済みとなる。

`pending_obstacle_cells.csv`には、通過前に保守側でobstacleが黒、確認済み側では黒でない
セルだけ保存する。複数の歴史的high trackから窓内positive最多の代表を選び、
対応窓の充填数・positive／valid no-hit／plane unknown、直近N画像でのセル観測数、
通過前全期間と初回hit以降のセル観測画像数を分ける。未対応画像をhistoryへ補充しない。
これらは確認待ちの原因診断で、candidate対応の同一物体保証ではない。

`analyze_height_footprint_causes.py --reference ...`は異なるNの場合に共通のN1／raw幅を照合。
同じNの再試なら全共通方式を照合する。以前の全方式が再現された意味と混ぜない。
同trackの初回確認遅れを比較する場合：

```bash
python3 src/pm_evaluation/tools/compare_height_confirmation_windows.py \
  /tmp/height_candidate_sequence_n5 \
  --short-window-directory notes/reports/perception/20261006_231622_高さ候補のground診断と短期窓比較_assets/sequence
```

同track ID・XYセル・初回positiveを検査し、違えば停止する。`tracks.csv.gz`にも対応。
確認済みになった母集団が違うN3／N5の平均遅延だけを見比べるより、共通trackの追加待ち時間を
確認する。CSVの出典stampは初回確認と初回hitを区別し、未確認をsafeへ落とさない。

## 確認待ち・確認済みを残したまま短期再観測で黒を解除する試験

`tools/evaluate_obstacle_window_clearance.py`は別のオフライン実験で、通常mapperや
上記の解除なし診断を変更しない。同じ採用画像・実使用TFで、解除なし対照と
N=1／3／5を一括計算する。各黒の原因、解除前の確認状態も保存する。

```bash
python3 src/pm_evaluation/tools/evaluate_obstacle_window_clearance.py \
  bags/rosbag2_2026_07_26-09_17_38 \
  --mapper-trace notes/reports/perception/20261003_100000_depth融合_stride比較_assets/stride4/trace/mapper_trace.jsonl \
  --output /tmp/obstacle_window_clearance_new
```

平均する高さは、各frame・各XYセルの全投影点について計算した
`h=max(0, max(z_point - z_provisional_plane(x,y)))`（m）。保存候補枠外の点も含む。
**以前のground候補／物体候補のz_meanの短期平均とは別の量**である。
以前の平均は候補高さの変動診断に使い、ground平面やterrainを平均値から再fitしたり、
障害物の黒を解除したりしていなかった。この実験でもground／terrain三指標は変更しない。

- 新しい高さがobstacle limit以上なら直ちに黒。古い低い高さで薄めない。
- 新しい黒episodeの開始時に窓をリセットし、その後の有効なセル観測を直近N個保持する。
  N個が揃い、今回の高さがlimit未満、平均がlimitの99.5%未満なら黒を解除する。
  99.5%は整数costの丸めで100となる境界に合わせるため。
- 解除まで確認待ちも確認済みも黒のまま。確認済みへの昇格は同一候補trackの
  直近N対応観測が全てhigh-hitで、全て今回の黒開始以降の場合。昇格後は解除まで保持。
- 解除窓は**有効なセル観測**、確認窓は**対応候補観測**であり、同じ分母ではない。
  画像欠測・平面unknownを0として窓へ入れない。時間での破棄もしない。
- 解除後に新しいhigh-hitがあれば再び即時黒にする。

`summary.json`に黒／unknown／全cue有効率、原因別footprint件数、処理時間、CPU時間、
peak RSSを保存する。`footprints.csv`は通過前の評価、`black_cells.csv`は黒の根拠値、
`clear_events.csv`は解除前状態・現在高さ・平均・観測数・黒開始からの秒数を保存する。
時間は分析結果であり破棄条件ではない。footprintは0.45×0.55mの暫定矩形。

**低い再観測はfree-ray証拠ではない。** 疎な画素、遮蔽、ground基準の変化でも
高点が消える可能性がある。真の障害物の誤解除や検出率をこの舗装路bagだけでは評価できず、
この規則を安全な走行判定として導入した意味にはしない。特に壁／枝の表面を同一セル内で
平均化する試験であり、サブクラスタの同一性に基づく高さ平滑化の完成版ではない。

## 付録用の立体HTMLデモ

`tools/demo_height_candidate_scene.py`は現在frameの投影点、ground／thin／broad／sparseの
候補外包、対応候補の平均z、N=1／3／5の保持hazardと確認状態を自己完結HTMLへ保存する。
ブラウザで視点回転・拡大・snapshot再生・N切替・セル選択ができ、ROS node／実機センサは起動しない。
外部CDNやWebサーバも不要。通常mapperは変更しない。

```bash
# runtime overlayをsourceし、source treeの新規moduleを優先する（ビルド不要）。
export PYTHONPATH=/workspaces/patasmonkey_ws/src/pm_evaluation:/workspaces/patasmonkey_ws/src/pm_perception:$PYTHONPATH
python3 src/pm_evaluation/tools/demo_height_candidate_scene.py \
  --synthetic --output /workspaces/patasmonkey_ws/notes/height_demo_synthetic_new.html

python3 src/pm_evaluation/tools/demo_height_candidate_scene.py \
  bags/rosbag2_2026_07_26-09_17_38 \
  --mapper-trace notes/reports/perception/20261003_100000_depth融合_stride比較_assets/stride4/trace/mapper_trace.jsonl \
  --start-seconds 20 --display-every-seconds 0.5 --max-display-frames 60 \
  --output /workspaces/patasmonkey_ws/notes/height_demo_july_new.html
```

start-secondsはtraceの最初の採用depthからの秒数。その前の画像も更新してから表示開始する。
表示snapshotは間引くが、その間の採用画像も全て候補・確認・解除の更新へ入れる。
`--max-display-points`（既定1800）は描画のみの間引き、pixel_strideはtraceの値を使う。
`--view-radius`（既定4m）は描画範囲のみで、状態の破棄条件ではない。
`--max-display-frames`（既定60）に達すると終了。長区間はHTML／メモリ所要が増える。
既存HTML／sidecar JSONは上書きしない。

箱は観測点の外包で、内部全体のoccupied証拠ではない。高い枝を通過可能とは判定しない。
候補の箱は今回frameだけ、hazardは過去の評価保持も含む。青い線は候補高さ平均で、
黒解除に使う最大平面残差平均とは別。白も部分unknownを含み、安全保証ではない。
人工例は説明用で実センサ性能を示さない。車体は暫定寸法の目安で実URDFではない。
