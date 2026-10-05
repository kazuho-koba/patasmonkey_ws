# 同一frameの地面基準obstacle診断：仮説②と方式E

`diagnose_frame_surface_obstacles.py`は、独立frame評価＋空間保持の既存処理に
比較storeを追加するオフライン専用script。通常perceptionや高さ時間融合を変更しない。
terrain三指標・TF・採用stamp・samplingは共通とし、obstacleだけ以下に交換する。

| 方式 | 定義 |
|---|---|
| baseline | セル内z最大−最小。従来の低高さ幅による0更新を含む |
| quantile_span | セル内zのq90−q10 |
| detrended_span | 同一frameの局所地面平面を差し引いた各点の残差最大−最小 |
| surface_q90 | 同一frameの局所地面平面に対する各点残差q90、負値は0 |

地面候補はセルzのq10。既存terrain feature関数で3×3近傍を通常の最小二乗fitし、
少なくとも5有効セル・非退化XY配置が必要。平面fit自体はRANSAC等のrobust推定ではない。
candidateは2点以上でのみ有効、plane方式はplaneがなければunknown。
unknownで過去の評価を消さないが、時間による失効はしない。分位点は線形補間。

単一frame内の地面基準による**原因切り分け候補**であり、真の障害物／ground分離や
free-space保証は実装していない。thin obstacleを分位点で消す危険や障害物面をgroundに
fitする危険が残る。空間的まとまり・visibility・動的物体の検証は別途必要。

Foxyコンテナ内でROS・外部workspace・本workspaceを順にsourceして実行する。
build不要。radius/max-depth/strideは既存空間評価と同じCLIで変更できる。

```bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python3 src/pm_evaluation/tools/diagnose_frame_surface_obstacles.py \
  bags/rosbag2_2026_07_26-09_17_38 \
  --mapper-trace notes/reports/perception/20261003_100000_depth融合_stride比較_assets/stride1/trace/mapper_trace.jsonl \
  --output /tmp/new_surface_test/radius5 \
  --radius 5 --max-depth 5 --projection-map-size 22 --stride 4 --single-frame-obstacle
```

2/5/10 mの3条件を実行し、以前の同条件assetsと照合して集計する。

```bash
python3 src/pm_evaluation/tools/summarize_frame_surface_obstacles.py /tmp/new_surface_test \
  --reference-root notes/reports/perception/20261005_100225_半径depth上限stride比較_assets
python3 -m unittest discover -s src/pm_evaluation/tools -p test_frame_surface_obstacles.py
```

`black_cell_evidence.csv`は通過経路セルにおける通過前のraw obstacle黒更新。
min/maxの元画素u/v、axial depth[m]、odom XYZ[m]、元画素周囲3×3の有効depth中央値を
保存する。近傍中央値との差は外れ値だけでなく物体境界・斜面でも生じるため、
外れ値確定の判定には使わない。samplingに選ばれた点の極値であり全raw画素の極値ではない。

各candidateの通過CSVと`analysis.json`に、unknown・原因別黒・raw黒の再分類を分けて保存する。
baselineの全通過CSVと集計値が既存評価と一致しなければ集計は停止する。

代表例のdepth・高さ分布・近い撮像時刻のRGBを保存する場合：

```bash
python3 src/pm_evaluation/tools/plot_frame_surface_obstacle_examples.py /tmp/new_surface_test/radius5 \
  --bag bags/rosbag2_2026_07_26-09_17_38 \
  --mapper-trace notes/reports/perception/20261003_100000_depth融合_stride比較_assets/stride1/trace/mapper_trace.jsonl
```

RGBは時刻が最も近い画像を文脈用に表示するだけで、depthとの画素対応を示さない。
