# 7月bag terrain hazard provenance診断

対象は `rosbag2_2026_07_26-09_17_38`。localizationは既存の最新localization replay構成を使い、bagに記録された古い`/wheel/odometry`・`/vio/odometry`は入力odomとして再利用せず、replay時に再計算した`/odometry/local`を経路評価に使用した。

## 追加した記録

通常の実機runtimeでは無効のまま、launchの`terrain_forensic_output_dir`を明示したoffline replayに限り、以下をCSV保存するようにした。

- depth画像から4×4間引きで得たraw pixel座標`u,v`とaxial depth値。各XY cellに入ったframe内sample数、world-z最小/最大候補と各候補のsource pixel/depth。
- cellのground elevation融合前後値、relative elevation融合前後値、fusion mode、最後にsourceとなった画像時刻、その時刻のbase pose。
- hazard黒セルのslope/roughness/step/obstacle値、局所平面`a,b,c`、fresh support数、残差RMS/min/max。
- plane fitに使われた各fresh近傍supportについて、cell座標、age、relative elevation、plane residual、元pixel/depth要約、観測時base pose。

全画素・全履歴・PointCloud2を保存する方式ではない。通常実行ではprovenance配列の割当、CSV IO、追加ROS messageを行わない。

## 対象地点・対応精度

先行した全体ROI記録と経路評価の完全一致では、前方約2mで見た中心黒地点37点中30点が同一map timestampかつ同一absolute odom cellとして対応した。残る7点は広ROI端・旋回部などで厳密対応しないため、cue統計へ混ぜていない。

さらに中心黒地点の近傍を重点記録する再生を実行し、raw pixel provenanceを含む`hazard_cells.csv` 192行、`plane_support.csv` 187行を得た。先行runのpath CSVとの完全一致は21/37点。pass間でmap stamp/odom gridが一致しない点が残るため、この21点の詳細provenanceは代表的な確定例であり、37点全ての生depth診断とはみなさない。重点再生の出力には通常runnerの最終path summaryがなく、重点CSVの取得範囲をbag全体と保証できないため、全体cue統計は完了済みの先行ROI replayを基準とする。

## 観測された内容

先行ROI記録で完全一致した中心黒30点ではcue閾値超過が重複していた。

| cue | 閾値以上の中心セル数 | 観測値の中央値 |
|---|---:|---:|
| slope | 20/30 | 27.21° |
| roughness | 17/30 | 3.59 cm |
| step | 21/30 | 10.53 cm |
| obstacle | 4/30 | 26.67 cm |

最大寄与cueの組み合わせでは、slope+roughness+stepの3項同時超過が12点、slope単独が7点、step単独が3点などだった（cueは排他的ではない）。このため、単一指標だけがhazard全体を作ったのではない。

重点記録で完全一致した21点のplane gradient中央値は38.50°、90 percentileは71.51°。それに対してセルを最後に観測したbase姿勢のroll中央値は1.40°、pitch中央値は-0.89°だった。姿勢の瞬時傾きだけでは、観測された平面gradientの大きさを説明できない。ただしodom姿勢・extrinsic誤差が過去supportの融合へ与えた寄与は別途残る。

同じ2D cellに1枚のdepth frameから入ったsampleのworld-z幅は、30点ROI一致では中央値1.61 cm、90 percentile 3.61 cm（最大25.71 cm）。一方、fresh plane support点と局所平面の絶対残差は中央値2.40 cm、90 percentile 8.33 cm、最大48.46 cmだった。つまり多数セルでは同一frame・同一cell内のraw候補はまとまっているのに、近傍セル全体は一貫した平面になっていない。これは単なる各pixelのランダムな散らばりだけでは説明しにくい。

具体例のraw行では、ある近傍cellの今回の最小world-z候補は約-0.161 m（pixel `(392,340)`, depth約2.011 m）だったが、過去から保持されたgroundは約-0.405 mで、差がground merge threshold 0.20 mを超えたためfusion mode 0（候補を融合しない）となり、低いground仮説が維持されていた。隣接cellはground約-0.196 m、relative elevation約-0.066 mであり、cell間に約22 cmの相対標高差が残る。この差は5 cm grid上では著しい擬似step/slopeを作り得る。

これはground仮説の履歴が黒hazardを作る経路を実データで確認したもの。ただし、その過去の低い観測がdepth外れ値なのか、撮像時pose/TF・カメラextrinsic・動体/地物によるものかは、この記録だけで最終断定できない。今回のCSVは各cellの最後のframe min/max代表sampleを保存するもので、全pixel履歴やRGB画像そのものは含まない。pixel/depthと画像を照合する場合は、`latest_source_stamp_ns`と`frame_min/max_pixel_*`を使い、同じbagのcolor/depth frameを確認する必要がある。

## 診断結果と次の切り分け

現時点で最も強い実装上の疑いは、低いcandidateをgroundとする最小z方式と、その後高いcandidateを0.20 m超の差として無視するfusion方針が、誤った低groundを長期間残し、近傍面を不連続にしていること。姿勢の瞬時roll/pitchはhazard slopeの大きさと整合せず、同一frame内の大半のcell raw spreadも小さい。一方で、過去姿勢の重ね合わせ誤差・depth外れ値が「最初の低ground」を作った可能性は未分離であり、perception妥当性を回復したとはまだ言えない。

次は、報告中の具体セルについてRGB/depth画像をtimestamp・pixel座標へ戻して、(1)当該pixelがアスファルトか物体/境界か、(2)neighbor supportが何時・どの姿勢から蓄積されたか、(3)同じ静止地面の再観測時にcandidateがどの程度ずれるかを照合する。続いて、ground hypothesisの上方追従/外れ値拒否を見直し、修正前後で同じ対象地点のraw pixel・candidate・fusion mode・cue値を比較する。

## 成果物

- `src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/provenance_1x_retry1/forensic_summary.json`
- 同ディレクトリの`matched_center_black.csv`、`matched_center_black_support.csv`、`forensic/hazard_cells.csv`、`forensic/plane_support.csv`
- `src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/provenance_targeted_1x/forensic_summary.json`
- 同ディレクトリの`matched_center_black.csv`、`matched_center_black_support.csv`、`forensic/hazard_cells.csv`、`forensic/plane_support.csv`

## 検証

`pm_perception`の直近のFoxy build/testは成功（14 tests）。今回の集計ツール変更後はPython compile、replay shellの`bash -n`、`git diff --check`を実行した。summarizerは旧support CSV（姿勢列がない形式）と新しいCSVの両方を処理できることを実データで確認した。
