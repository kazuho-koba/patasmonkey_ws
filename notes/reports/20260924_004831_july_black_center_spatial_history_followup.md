# 7月bag 黒判定中心セルの空間履歴・未検証項目の追加調査

作成: 2026-09-24 00:48:31 JST  
対象bag: `rosbag2_2026_07_26-09_17_38`  
再生速度: 1.0倍  
関連する前回レポート: [20260923_235834_july_hazard_followup_hypothesis_tests.md](20260923_235834_july_hazard_followup_hypothesis_tests.md)

## 要約

前回の調査では、黒くなった走行中心セルを代表時刻で切り出し、その時点でのplane supportやground融合前後を確認した。今回はその37セルをodom上の固定セルとして追跡し、再生中にrolling map上で確認できた履歴を追加した。

結論は、すべての対象セルについて履歴を取得でき、2 Hzのmap debug snapshotで観測された365件の異なるdepth入力イベントのうち、4件が既存groundを20 cm超えて下げる`lower replace`だった。3件は同じセル`(67, 1)`、1件は`(152, 124)`である。したがって、数十cmのセル変動は多数のセルで頻発したのではなく、少数のtail caseだった。一方、その4件では、フレーム内候補点の垂直幅、画像上の候補位置、fusion前後の値に不連続があり、単純なローパス平均だけで解決できるとはまだ言えない。

特に`(67, 1)`の3件は、base zが0 m、roll/pitchもほぼ一定の短時間内に発生しているため、少なくともこの3件は「その瞬間のodom zジャンプ」だけでは説明できない。depth値3.351 mの候補画素は3フレームでv=156、248、280へ移り、画像上の同じ路面点を追跡しているとは確認できなかった。カラー画像では候補の投影位置が壁・建物側から路面側へ移って見えるケースがあり、同じ地面面を比較できていない可能性が高い。ただし画像の単眼目視だけで、実際の3D表面対応を確定したわけではない。

これに対して`(152, 124)`の1件は、融合前後の相対groundが+0.209 mから-0.037 mへ約24.6 cm変わり、同時にbase z=-0.137 m、pitch=-1.21°、yaw=68.22°だった。姿勢推定・depth候補・セル内surface混在の寄与を分離できておらず、姿勢が原因と断定できない。

よって現在の証拠は、「通常更新の大半は小さいが、まれな大変動イベントが存在する。セル内に異なる面の候補が入ることと、pose/depth投影の誤差の双方が候補。どちらが主因かは未確定」である。

## 前回レポートとの関係

前回は主に優先度1〜3として、カメラintrinsics/distortionの影響、黒中心セルのsnapshot、plane support、代表イベントの画像確認を扱った。本レポートはsnapshotから時系列へ範囲を広げ、代表的な2セルだけでなく黒中心対象37セル全体を2 Hz debug snapshotで追った追加検証である。

- 前回の「セル高さ変動・ground高さ変動」は代表snapshot間の比較が中心で、depth callbackごとの完全な履歴ではなかった。今回は同一odom cellの更新を時系列化したが、記録周期はmapperのdebug map publishに合わせた約2 Hzであり、最大10–12 Hzのdepth callbackを網羅してはいない。
- 前回のdistortion感度試験では、D補正によるworld-z変化は大半が数mm〜約1 cmで、代表セルの約29 cm差を説明しなかった。今回もDを変えて再生するA/B比較はしていないため、Dが各4イベントに与える正確な差は未検証。ただし過去の数値から主因である可能性は低い。
- 前回の姿勢原因切り分けは未完了だった。今回、特定の3イベントではbase zとroll/pitchが安定していたことを確認できたが、もう1イベントやbag全体についてpose要因を排除できたわけではない。
- この調査は融合閾値・候補抽出・地形cueを変えたA/B試験ではない。現在のblack/hazard判定が物理的に正しいかを最終検証するものではない。

## 検証方法

1. 既存の1倍速オフラインrunnerで7月bagを再生し、分離した最新localization出力をmapperへ入力した。bag中の古いodometryをそのままmapperに使う試験ではない。
2. 前回の`path_samples.csv`から、走行中心がhazard=1だった37サンプルのodom上のセルを抽出した。37サンプルは37個の異なる空間セルだった。
3. forensic targetの時刻指定を空にし、各固定odom cellがrolling map内に存在する各map snapshotで、最新source stamp、fusion mode、ground融合前後、同一depth frame内の候補zのmin/max、pixel座標、pose、hazard cueを記録した。
4. 2 Hz履歴で見つかった4件のlower-replaceについて、bagのdepth/color画像をsource pixel位置つきで抽出し、目視できるカラー画像とdepth近傍統計を確認した。

## 結果

### 1. 黒中心37セルの更新履歴

- 対象: 37セル
- 2 Hz map snapshotで履歴を取得できたセル: 37/37
- 対象セルのsnapshot行: 2,023行
- snapshotで観測できた異なるsource event: 365件
- fusion mode内訳: weighted merge 265、初期化 54、上側候補の棄却 42、20 cm超のlower replace 4
- lower replaceの既存値からの低下量: 最小21.0 cm、中央値25.7 cm、最大53.7 cm
- lower replaceが起きたセル: `(67, 1)`に3回、`(152, 124)`に1回

これは「37セル全部が何度も数十cm揺れた」という結果ではない。ほとんどはweighted mergeで、該当4件が少数の大きなtail caseを構成した。ただし上記365件は2 Hz時点で見えたsource event数であり、深度入力10 Hz前後の完全なイベント数ではない。

### 2. `(67, 1)`の3 lower-replace

約0.9秒の間に、既存groundが候補によって以下のように更新された。

| source pixel `(u,v)` | depth | ground更新前→後 | 相対ground更新前→後 | frame内候補数 | frame内z幅 | capture base z / roll / pitch |
|---|---:|---:|---:|---:|---:|---|
| (352,156) | 3.351 m | 0.842→0.593 m | 0.962→0.713 m | 10 | 28.1 cm | 0 m / 1.63° / 0.55° |
| (356,248) | 3.351 m | 0.593→0.056 m | 0.713→0.176 m | 10 | 58.5 cm | 0 m / 1.63° / 0.55° |
| (352,280) | 3.351 m | 0.080→-0.130 m | 0.200→-0.010 m | 1 | 0 cm | 0 m / 1.63° / 0.56° |

最後のイベントの`ground更新前=0.080 m`は直前のlower-replace後の0.056 mと一致しない。間にmap snapshotされなかったsource updateが存在するか、debug snapshotが各depth callbackを記録しきれていないことを示す。したがって2 Hz記録から全融合順序を復元することはできない。

3フレームともsource pixelのdepth値は3.351 mだが、pixelのv座標は大きく異なり、33×33 pixel近傍のdepthも複数距離に広がっていた。たとえば最初の候補近傍は3.351 mと5.026 mに分かれ、2件目では1.587〜4.308 mに広がっている。単一候補点が一貫した路面パッチを代表しているとは言いづらい。

カラー画像のsource pixel markerは、旧候補と新候補が常に同じ可視路面点を表しているわけではないことを示唆した。特定フレームではmarkerが壁・建物側、次のフレームでは路面側に見える。これは複数面の混入仮説を支持するが、画像上のpixel間に対応付けを行っていないため、面の誤分類を確定した証拠ではない。

この3件ではodom base zは0 mで一定、roll/pitchの差もごく小さい。ゆえに、これらの大きな候補差を「その時刻のbase zジャンプ」や大きなroll/pitch変化だけで説明することはできない。depth点が別面を拾った可能性、pixel/depthの投影とセル割当て、より微細な姿勢・TF誤差は残る。

### 3. `(152, 124)`の1 lower-replace

- source pixel: `(168,316)`、axial depth 3.016 m
- frame内候補点: 2点、z幅2.1 cm
- ground: -0.030 m → -0.294 m（-26.4 cm）
- 相対ground: +0.209 m → -0.037 m（-24.6 cm）
- capture pose: base z=-0.137 m、roll=1.67°、pitch=-1.21°、yaw=68.22°
- この時点の最大cue: slope

このフレーム内では候補z幅が小さいため、同一フレーム内での大きなばらつきが直接原因とは見えない。一方、異なるフレーム間で地面候補の高さが大きく変わった。直前観測からの姿勢・位置・depth rayの差、地面でない面の選択、セル投影のいずれも残っている。poseのzだけで説明できるかは、直前と当該フレームのraw TF/IMU、depth rayを同じ3D点対応で比較しないと判断できない。

### 4. frame内の大きなz幅は別のtail caseにも存在

2 Hzで見えた365件のframe candidateについて、同じXYセルへ入る候補点の`max z - min z`を確認すると、10 cm以上が6件、20 cm以上が4件あり、最大は1.392 mだった。これは異なる鉛直面が同一XYセルに入る、またはdepth/pose/セル投影に異常がある候補である。

重要なのは、この最大z幅を今回の黒中心37セルすべてに一般化しないこと、またz幅だけで原因を「壁」と断定しないこと。セル内vertical surface mixingを処理できていない可能性を示す診断量であり、最大値が実際に地面hazardを決めたかは別途source画像とcueの因果追跡が要る。

## いま言える原因評価

1. **通常観測の小さなノイズが平均で残っただけ、という説明だけでは不十分。** Weighted mergeが多い一方、20 cm超の別modeで低い候補へ切替わるイベントが確認された。
2. **このlower-replaceは少数のtail case。** 見えた365 event中4件であり、全セルにわたる定常的な数十cmぶれではない。ただしdebug snapshotで観測された範囲に限る。
3. **少なくとも`(67,1)`の3件をodom zジャンプ単独では説明できない。** 該当capture base zは0 m、roll/pitchは安定している。画像上のsource位置と近傍depthは複数面/距離の混入を疑わせる。
4. **`(152,124)`は姿勢推定も依然候補。** base zが-13.7 cm、pitch/yawも別値だが、地面rel-zの変化はbase z差だけではなく、姿勢を真因と特定できる証拠ではない。
5. **平均化を先に入れるのは危険。** 外れ値候補が壁・障害物なら単純な時間平均は別surfaceをgroundに混ぜ、地面境界をぼかす可能性がある。まず候補の空間的支持、近傍depth分布、画像面対応、pose rayを検証すべき。

## まだ未検証／不足していること

- 2 Hz forensic outputは全depth callback（約10 Hz）を記録していない。lower-replaceが4件より多い可能性、およびsnapshot間の中間ground stateは未確認。
- distortion Dを有効にした全mapper A/Bは未実施。以前のray感度計算はD補正が各world-z候補に与える幾何差の見積りであり、候補選択・セル割当て・hazard結果を再演算した比較ではない。
- local planeのsupport/残差が、lower-replace直前直後およびblack判定にどう寄与したかを、全履歴について定量比較していない。
- black center区間を連続したカラー特徴として追跡し、独立なvisual motion/姿勢変化とodometryを比較する検証は未完了。静止画像の目視だけでは姿勢誤差と誤ったsurface候補を分離できない。
- 同一路面の画像/深度点対応を作り、poseを固定した再投影と、depthを固定したpose差分をそれぞれ計算するcounterfactualが未実施。
- hazard=1となった後の滞在・回復履歴と、どのcomponent cueが何フレーム持続したかの全対象セル統計が未完成。

## 次の検証候補（優先順）

1. **diagnosticをdepth callbackごとのイベント記録にする（オフラインのみ）**: 対象37セルだけを絞り、source stamp・candidate pixel/depth・candidate数・z min/max・fusion前後・mode・poseを1 depth frameにつき記録する。production runtimeの常時debug負荷を増やさないよう、offline runner専用とする。これにより2 Hz snapshot間の更新順を確定する。
2. **4 lower-replaceと同じセルの前後フレームを3Dレイ単位で比較**: pixel markerだけでなく、local ground planeへ投影した点、近傍支持点、depth分布、plane residualを出す。候補が路面/壁/別物体のどれかを判別する。
3. **pose counterfactual**: raw candidate depth/pixelを保持し、同じTF姿勢で再投影する場合と各capture TFで再投影する場合を比較する。base zだけでなくroll/pitch/yawとcamera extrinsicsも含め、`(152,124)`の24.6 cm変化をどの成分が説明するか確認する。
4. **ground候補selectorの限定A/B**: 時間平均ではなく、近傍平面支持数・残差・depth連続性を満たす候補だけをground更新に使う試験をoffline replayで行う。障害物cueの保持を壊さないことも合わせて確認する。
5. **D補正full A/B**: distortionを反映したrayでpoint-to-cellとhazardまで再計算する。ただし以前の数値からは優先度低めで、まず上記1〜3を行う。

## 生成した解析データ

- 空間target生成: `src/pm_evaluation/tools/create_black_center_spatial_targets.py`
- 2 Hz sampled history集計: `src/pm_evaluation/tools/analyze_black_center_history.py`
- 対象出力: `src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/black_center_history_1x/`
- depth/color source画像照合: 同フォルダの`replace_images/ground_source_contact_sheet.png`、`ground_source_image_matches.json`

## 再現コマンド

```bash
python3 src/pm_evaluation/tools/create_black_center_spatial_targets.py \
  src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/provenance_1x_retry1/path_samples.csv \
  src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/black_center_history_1x/spatial_targets.csv

python3 src/pm_evaluation/tools/analyze_black_center_history.py \
  src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/black_center_history_1x/spatial_targets.csv \
  src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/black_center_history_1x/forensic/hazard_cells.csv \
  --output-dir src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/black_center_history_1x
```

ROS replayは既存`pm_evaluation`の1x terrain replay runnerを使用し、`TERRAIN_FORENSIC_TARGETS_CSV`に生成した`spatial_targets.csv`を指定した。対象bagの再計算localizationを使う既存構成を維持し、実機・車両を動かす操作は行っていない。
