# 時間融合なしのdepthフレーム独立hazard診断

前回の`terrain_obstacle_review.launch.py`・`terrain_obstacle_review.yaml`は
変更せず、通常の融合＋RViz表示に引き続き使う。この診断は別ツールであり、
センサ・モータを起動しない。MCAP形式のbagを対象とする。

## 何を分離するか

### unknownと地形support不足の追加診断

`--footprint-cell-diagnostics`を指定すると、frame/経路CSVへfootprint内の各セルの
odom XY、各cue、support数、現在frameのmin/maxをJSON列として保存する。
既定OFFであり、通常mapperには影響しない。独立と時間融合の両実行で同じ
pose CSV・設定・strideを使い、異なる新規出力先を指定する。

```bash
python3 src/pm_evaluation/tools/analyze_paired_terrain_support.py \
  /tmp/independent_with_cells /tmp/temporal_with_cells \
  --output /tmp/paired_support_new
```

独立側の`reference_hazard`と時間融合側の`hazard`を同じ撮像stamp・通過stamp・
セルXYで照合する。hazard共通既知、平面共通成立、地形3cue共通成立のmaskを別に集計する。
共通maskの率はfootprintの一部だけを対象にする場合があるため、coverageを併記し、
車体全域の安全率と解釈しない。XY/stamp不一致は補間せず拒否する。

depthは撮像時刻のcamera→base static TFと、最新localizationで再計算した
base→odom姿勢を使って投影する。毎フレーム新しい空gridを作り、同じXYセルの
全pixelから最小zをground候補にする。別時刻のground観測は混ぜない。
そのgridを**既存のcompute_terrain_features**へ渡し、slope・roughness・stepを
同じ計算式で評価する。既定stride=1で、pixel間引きは行わない。
無効depth・距離範囲外・map範囲外の点は従来同様に除外する。

現行obstacle confidenceは1画像で0.1、最低採用値は0.15なので、通常は1枚だけでは
obstacleが有効にならない。現行ルールの`hazard`と、confidenceを使わず
単一画像のセル内max−minをobstacle cueにした`reference_hazard`を別々に保存する。
`--single-frame-obstacle`を指定すると後者を主な評価指標として選ぶ。
セル内max−minが`obstacle_min_height`以上なら証拠を使い、
`hazard_obstacle_height_limit`以上ならobstacleだけでhazard=1にできる。
最低記録高さを満たすだけではhazard=1にならない。
両方の結果は常に保存するため、オプションなしの既存診断結果でも後者を確認できる。
後者は**confidence gateを変更した診断指標で、現行hazardと同じアルゴリズムではない**。
unknownを安全へ置換しない。単一画像のsparseなsupportでは、各指標がunknownになる。

`--temporal-fusion`は対照用にgridを保持する。独立／時間融合の両条件で、同じ
depth列・同じpose列・同じstride・同じ閾値を使う。DDS受信・TF待ち・rate gateは
オフライン処理にはなく、hazardを各画像で計算する。実機runtime 2 Hzや前回の
2 Hz地図結果と完全同条件ではないため、因果比較はこのツール内の対照間で行う。

## 姿勢の再計算（Foxyコンテナ）

新しい専用launchをインストールするため、最初だけ`pm_perception`をbuildする。
build時はmain overlayをsourceしない。

```bash
cd /workspaces/patasmonkey_ws
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
colcon build --packages-select pm_perception --symlink-install
source /workspaces/patasmonkey_ws/install/setup.bash
bash src/pm_evaluation/tools/run_single_frame_pose_replay.sh \
  /workspaces/patasmonkey_ws/bags/rosbag2_2026_07_26-09_17_38 \
  /tmp/single_frame_july
```

latest localizerだけを1倍速で再計算し、`recomputed_poses.csv`を保存する。
元bagの旧odom・旧動的TFは再生しない。撮像時刻に位置線形補間・quaternion SLERPを
行う。poseがない時刻や0.25秒を超える補間区間は棄却し、件数を報告する。
姿勢を変更したい場合は新しいpose CSVを生成する。手元の古い解析CSVを黙って使わない。

## 全フレームの独立評価

```bash
python3 src/pm_evaluation/tools/evaluate_independent_depth_frames.py \
  bags/rosbag2_2026_07_26-09_17_38 \
  --poses-csv /tmp/single_frame_july/recomputed_poses.csv \
  --config src/pm_perception/config/depth_elevation_mapper_hazard_0p05_baseline.yaml \
  --override src/pm_perception/config/single_frame_terrain.yaml \
  --stride 1 --single-frame-obstacle \
  --output /tmp/single_frame_july/independent_stride1
```

時間融合対照は`--single-frame-obstacle`を外し、`--temporal-fusion`を加え、
**別のoutput**を指定する（1枚のconfidence省略と時間融合履歴の変更を混同しない）。
strideによる差は`--stride 4`で別出力へ保存する。`--resolution 0.05`等も指定可能。
今回用設定`single_frame_terrain.yaml`は既定セル幅0.10 m、obstacle記録0.03 m、
hazard obstacle限界0.08 m。他の条件は比較baselineを用いる。

## 分母と経路評価

各frameのmap上で、その後0.2〜12秒内に実際に通った位置を調べる。
撮像時の車両から約2 m前方（±0.25 m）となる経路位置を対応付ける。
footprintは幅0.45 m×長さ0.55 m、通過時のyawを使う。

- `frame_weighted`：各frameにつき距離誤差最小の通過位置を1つ選ぶ。
  観測周期・停止時間・低速走行による重みが入る。
- `distance_weighted`：従来と同様に、前の選択経路位置から0.25 m以上進んだ
  通過位置を選び、その位置について最良距離のframeを1つ採る。
  同率なら新しいframeを採る。こちらを主な比較指標とする。
- 分母はgridセル総数ではなく、対応が取れた**通過位置／footprintのサンプル数**。
  フレームを増やしても距離ベースの分母は同じ経路なら大きく増えない。
- 黒率：既知セルがあるfootprintのうち値100のセルが1つ以上ある割合。
  従来表示と合わせて0〜100丸め後の100を使う。
- 全unknown・部分観測・完全観測の件数と、完全観測footprintだけの黒率も出す。
  地図候補のない経路位置数も報告する。unknown率はfootprint内延べセル数ベース。

## 出力・可視化

`summary.json`、`frame_footprints.csv`、`path_footprints.csv`、
`frame_diagnostics.csv`を保存する。代表frameフォルダに`map.npz`（生値）、
`depth_mm.npy`、`map.png`を保存する。map画像はhazard黒=1・白=0、unknown=紫。
全frameの画像・点群は保存せず、オフラインでもメモリと出力量を抑える。

単に1枚だけを選んで見る補助ツールもある：

```bash
python3 src/pm_evaluation/tools/evaluate_single_depth_terrain.py \
  bags/rosbag2_2026_07_26-09_17_38 --seconds 30 \
  --config src/pm_perception/config/depth_elevation_mapper_hazard_0p05_baseline.yaml \
  --override src/pm_perception/config/single_frame_terrain.yaml \
  --roll-deg 0 --pitch-deg 0 --output /tmp/one_depth_frame
```

この1枚用補助ツールの姿勢既定値は**水平仮定**であり、最新odomの姿勢を自動取得しない。
原因究明の主評価にはpose CSVを使う全フレーム版を用いる。
1枚版は最も近いRGBも参考表示するが、depthとのpixel位置合わせは行わない。

## 通常mapperが実際に使用したTFで再評価する（推奨）

`replay_trace_output_dir`を指定したmapperは、融合完了した画像ごとに
撮像stamp・camera→map TF・base→map TF・使用intrinsics・寸法を
`mapper_trace.jsonl`へ保存する。回転はxyzw quaternionであり、評価側で再補間しない。
2 Hz snapshotの評価stampと最終融合画像stamp・融合枚数も記録する。
通常は空文字なのでJSON・ファイル生成をしない。画像・点群は複製しない。
既存ログの上書きを防ぐため、新規の出力先を指定する。

採取（Foxyコンテナ、最初にpm_perceptionをbuild・runtime overlayをsource）：

```bash
bash src/pm_evaluation/tools/run_mapper_tf_trace_replay.sh \
  /workspaces/patasmonkey_ws/bags/rosbag2_2026_07_26-09_17_38 \
  /tmp/july_mapper_tf_trace
```

別launch `mapper_tf_trace_replay.launch.py`は、通常mapperと最新localizationを
起動する。RELIABLE/history=3、最大処理25 Hz、hazard 2 Hz。比較baselineを基本に、
専用`mapper_tf_trace.yaml`でセル幅10 cm・obstacle 3 cm/8 cmを指定する。
これは現在のruntime YAML全項目を自動継承するという意味ではなく、検証用に固定した
条件で通常mapperを動かす。別条件が必要なら専用launchのmapper_config / trace_configを
明示する。前回のRViz用launch・YAMLは変更しない。

このlaunchも`pixel_stride`、`resolution`、`obstacle_min_height`、
`hazard_slope_limit_deg`、`hazard_roughness_limit`、`hazard_step_limit`、
`hazard_obstacle_height_limit`を同名引数で上書きできる。未指定なら専用YAMLの値を使う。
stride既定値は4（横・縦とも4画素おき）、hazard限界は順に20 deg、0.03 m、
0.07 m、0.08 m。component表示上限も各限界へ自動追従する。
設定変更後はlaunchの再起動が必要。runnerにも追加引数をそのまま指定できる：

```bash
bash src/pm_evaluation/tools/run_mapper_tf_trace_replay.sh \
  /workspaces/patasmonkey_ws/bags/rosbag2_2026_07_26-09_17_38 \
  /tmp/july_mapper_tf_trace_stride2 \
  pixel_stride:=2 hazard_slope_limit_deg:=15.0 hazard_step_limit:=0.05
```

この変更は通常mapperの採用画像・実効設定の採取に適用される。
独立フレーム評価側のpixel間引きは引き続き`--stride`で別途指定する。

同じ再生の実績経路CSV・実効parameterも保存する。runnerの終了まで待ってから評価する。
未完了traceにはcomplete recordがないため評価側が拒否する。

```bash
python3 src/pm_evaluation/tools/evaluate_independent_depth_frames.py \
  bags/rosbag2_2026_07_26-09_17_38 \
  --poses-csv /tmp/july_mapper_tf_trace/recomputed_poses.csv \
  --config /tmp/july_mapper_tf_trace/depth_elevation_mapper.yaml \
  --mapper-trace /tmp/july_mapper_tf_trace/trace/mapper_trace.jsonl \
  --stride 1 --single-frame-obstacle \
  --output /tmp/july_mapper_tf_trace/independent_actual_tf
```

`--mapper-trace`指定時は、投影にpose CSVの補間値やbag static TFの再合成を使わず、
記録したcamera→mapを直接使う。base→mapはrolling map中心とstepのheadingに使う。
pose CSVは将来通過位置の経路を評価する用途に残る。
採用記録にない画像は除外する。採用順序・画像stamp・寸法・map frameが合わない場合は
停止する。主要map/feature parameterがtraceと異なる場合も拒否する。
strideは意図した比較因子として変更可能。融合有無だけを見るには`--stride 4`等、
採取時と同じpixel_strideに揃える。

**現段階で一致するのは採用画像と投影TF/intrinsicsであり、評価周期ではない。**
この全frameツールは引き続き採用画像ごとにhazardを計算する。
snapshot評価時刻は保存しているが、2 Hz snapshotのage・地図選択を再現する機能は
まだ使っていない。従来ROSの黒率と完全同条件と誤認しない。

## 診断結果の限界

独立化で黒率が減っても、obstacle confidence履歴が消える影響と時間融合された
groundの影響を分ける必要がある。参考hazardを併記しても、物理的な障害物の真値には
ならない。単一frameでも姿勢誤差・depth外れ値・セル最小値ground候補・support不足は
残る。黒率が低いことだけでperceptionの正しさを証明することはできない。
