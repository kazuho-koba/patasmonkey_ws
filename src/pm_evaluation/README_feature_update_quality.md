# 観測品質と更新遷移のオフライン診断

通常の空間保持評価の更新規則を変えず、経路footprintセルの最後の通過前までの
品質と遷移を保存する。高さ時間融合や品質による更新選別は行わない。build不要。

## 実行

Foxyコンテナ内でROS・外部workspace・本workspaceを順にsourceする。

```bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python3 src/pm_evaluation/tools/diagnose_feature_update_quality.py \
  bags/rosbag2_2026_07_26-09_17_38 \
  --mapper-trace notes/reports/perception/20261003_100000_depth融合_stride比較_assets/stride1/trace/mapper_trace.jsonl \
  --output /tmp/new_quality/radius5_stride4 \
  --radius 5 --max-depth 5 --projection-map-size 22 --stride 4 --single-frame-obstacle
```

半径／depth上限を2／2、5／5、10／10 mにして3回実行する。出力directoryは新規にする。
集計scriptはこの3代表条件・stride4を対象とする。ホストPython＋NumPyでも実行可能。
通常空間評価の同条件assetsを比較先として指定し、診断が保持結果を変えていないことを確認する。

```bash
python3 src/pm_evaluation/tools/summarize_feature_update_quality.py /tmp/new_quality \
  --mapper-trace notes/reports/perception/20261003_100000_depth融合_stride比較_assets/stride1/trace/mapper_trace.jsonl \
  --reference-root notes/reports/perception/20261005_100225_半径depth上限stride比較_assets
```

## 出力・意味

- `quality_events.csv`：cueごとの受理更新。旧新の距離・点数・support・配置・残差・heading。
- `passage_black_origins.csv`：通過時の黒cueと、その連続黒episodeの起点。
- `quality_scope.json`：対象セル数・イベント数。通常のsummary／CSV／NPZも保存。
- `quality_analysis.json`：セル全体も非黒→黒となるcase、通常更新control、層別対照、起点集計。
- `matched_cases.csv`：対照を作れたcaseと、その旧新品質・対照率。

`safe`は既知・非黒の略で、安全の真値ではない。cue単体の黒への変化とセル全体の
hazardの黒への変化は異なるので、後処理で保持を再構成して区別する。

実測距離はterrain近傍patch／obstacle中心セルの点数加重平均。主診断の遠距離化は、
depth外れ値自身の影響を避けるためTFのcamera XYからセル中心への水平距離を使用する。
配置は同一frameの近傍XY共分散の固有値比・近傍重心の偏りを記録する。
残差RMSはroughness自体でもあるため、残差が大きい観測を自動で低品質としない。

層別対照は同じセル・cue、旧幾何距離0.5 m bin、旧support数、旧点数log2整数binを合わせる。
対照を作れなかったcase数も確認する。繰返し観測は独立試行ではなく、因果や有意性は保証しない。
診断flagは距離5 cm超増加、点数20%以上減少、support減少、固有値比0.1超低下、
重心偏り0.25 cell超増加、step両側support減少。obstacleには距離・点数だけを適用する。

品質悪化を伴う黒episodeの起点があっても、後で良い条件で黒が再確認された可能性がある。
本診断は更新規則のA/B効果や誤判定を確定しない。センサ・モーターは起動しないが、
イベント数・メモリ使用量は大きくなりうるため開発PCでのオフライン使用を想定する。
