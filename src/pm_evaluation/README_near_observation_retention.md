# 点数条件付き近距離優先のオフライン比較

品質診断の`quality_events.csv`を再生し、通常更新Aと近距離優先Bを比較する。
指標の計算は共通で、通常perceptionを変更しない。全mapではなく記録済み経路
セルの通過前評価専用。Aの各通過行が元評価と一致しなければ停止する。

Bは新距離が保持観測より5 cm超遠く、保持観測の点数が新観測以上なら拒否する。
terrainは3×3patch点数、obstacleは中心セル点数。距離は撮像TFからの水平距離。
解除も同規則で拒否されうる。未知のセルの初回観測は受理し、時間失効はしない。

Foxyコンテナ内でROS・外部・本workspaceを順にsourceして実行（build不要）：

```bash
python3 src/pm_evaluation/tools/compare_near_observation_retention.py \
  notes/reports/perception/20261005_103119_品質上書き診断_assets/radius5_stride4 \
  --mapper-trace notes/reports/perception/20261003_100000_depth融合_stride比較_assets/stride1/trace/mapper_trace.jsonl \
  --distance-margin 0.05 --output /tmp/new_near_comparison
python3 -m unittest discover -s src/pm_evaluation/tools -p test_near_observation_retention.py
```

出力は比較JSON、A/B各通過CSV。出力先は新規directoryを指定する。JSONの
changed_passagesに分類が変わった通過点、rejected_updatesに拒否と解除の件数を保存。
診断記録は選別前の全有効更新が必要で、既に品質選別された履歴は入力に使わない。
