# 7月bag: odom z再訪とterrain hazardの追加照合

## 何を比較したか

完了済みの最新localization replay結果`provenance_1x_retry1`を使い、`path_samples.csv`内で時刻差30秒以上、odom XY距離15 cm以内の経路点ペアを検索した。対応するlookahead map stamp・absolute cellがprovenance CSVにあれば、odom z、absolute ground、relative ground、hazard cueを同じ地点ペアで比較した。対象は平坦路を走った7/26 bagであるが、ここで「同じ地点」はodom XYが近いという意味であり、外部測量で地上の同一点と確認したわけではない。

## 直接観測できたこと

- 経路点425点のodom zは`-0.799 m`から`+1.714 m`まで、合計約`2.513 m`変化している。
- 30秒以上空いて15 cm以内のXYへ戻った候補は22 pair。最大のz差は`1.292 m`で、XY差`0.079 m`、時間差`84.29 s`だった。
- その両方のlookahead map cellについて、provenanceのhazard rowも照合できた。

| 観測 | 1回目 | 約84秒後 | 差/見方 |
|---|---:|---:|---|
| 経路点odom z | -0.108 m | +1.183 m | +1.292 m |
| map作成時odom z | -0.126 m | +1.208 m | +1.334 m |
| cellのabsolute ground after fusion | -0.318 m | +1.034 m | +1.352 m |
| relative ground after fusion | -0.0678 m | -0.0514 m | +0.0164 m |
| center hazard | 100 | 100 | 両方黒だが主cueは異なる |

odom zが大きく変わった一方、対応cellのabsolute groundもほぼ同量だけ上がり、relative groundの変化は約1.6 cmだった。これはmapperが

`relative_elevation = point_z_in_odom + nominal_camera_height - camera_z_in_odom`

で高さを正規化しており、depth点とカメラ位置に共通するodom z平行移動が概ね相殺される動作と整合する。したがって、この例は「zが1.3 m変わったため、そのままrelative elevationも1.3 mずれてslope/stepになった」という説明を支持しない。

## それでもz推定は重大な疑い

平坦路で推定XYが8 cm以内に戻った点のzが1.29 m違うのは、localizationの鉛直一貫性として極めて大きな不整合である。replay logには、`/vio/odometry`のzが1 frameで`1.727 m`変化し、設定した`0.25 m`上限を超えてvertical gateがlatchedした記録もある。最後のpath samplesではzが`-0.799 m`で複数点続き、gate後にheight observerが最後の値を保持する設計とも整合する。ただしlog時刻はbag header時刻とは別の時計なので、ここではこのeventを特定のcell更新と一対一に対応づけていない。

ゆえに「odom zは正常だった」とは言えない。zはabsolute elevation値とground fusion判定に影響し、閾値`0.20 m`をまたぐとcandidateを拒否したり、低いgroundへreplaceした差をobstacle heightとして積算したりする可能性がある。しかしrelative-elevationの共通mode補償があり、zの乱れがslope/roughness/stepの偽差へ直結するかは別に確認する必要がある。

## 黒判定の中身は再訪ごとに異なる

最初のlookahead cellはprovenance上で次の状態だった。

- slope `5.59°`、roughness `0.0142 m`、step `0.0552 m`はいずれも閾値未満。
- obstacle `0.2888 m`で黒。fusion mode 2（低いground candidateへreplace）。
- groundは融合前`-0.0289 m`から融合後`-0.3178 m`へ約`-0.2888 m`変わった。
- 同じcellのrelative groundは融合前`+0.210 m`、融合後`-0.0678 m`。つまりその更新ではabsolute zだけでなくrelative height仮説も約28 cm変化している。

2回目の近接cellはslope `14.50°`、roughness `0.0257 m`、obstacleはunknown、step `0.0736 m`だった。step閾値`0.07 m`をわずか`3.6 mm`超えたことでhazard=1になっている。こちらは第一例のようなobstacle差ではない。

この二点から言えるのは、同じ平坦路付近でも黒判定が一種類の再現性あるcauseではなく、1回目はground replacementに付随するobstacle cue、2回目は閾値近傍のstep cueだったこと。第一例のground差`28.9 cm`は、現在のcapture pose z`-0.130 m`がどれだけ動いたかだけでは説明できない。当時の初回CSVには融合前のsource姿勢・pixel履歴がなかったため原因は未確定だったが、後述の追加replayでその履歴を取得し、z並進だけでは説明できないことをさらに絞り込んだ。

## ユーザー仮説への回答

「zが跳ねると既存セルと新規セルのabsolute elevationにギャップができる」は十分ありうる。そのギャップがabsolute elevation mapに現れることは、今回の約1.35 mの差が示す。一方、現stageのslope/roughness/stepはrelative elevationを使うため、純粋な共通z平行移動だけなら理論上は相殺される。zはabsolute-ground fusionと、差分をobstacleへ変換するロジックには残るので、hazardへの影響はありうるが「map seamがそのままslope blackを作る」とはまだ言えない。

現データで最も妥当な結論は、(1)最新localizationのzは平坦路再訪間で非常に不整合、(2)relative elevationは共通zの多くを相殺している例がある、(3)別の観測間relative ground差が約28 cmあるセルがあり、fusion lower-replaceがその差をobstacle cueへ変えた、(4)その古いrelative ground候補の出所は未記録のため断定できない、の4点。

次の切り分けは、1回目のcell `(152,124)`の融合前groundを作ったsource frame姿勢・pixel/depthを履歴として追加し、現frameの`(224,356), 2.011 m`と比較すること。また、z gateがlatchedする直前後のraw VIO z、gated z、`/odometry/local` z、map relative elevationを同一bag timestampで記録し、(a)absolute groundのみが一緒に動くのか、(b)relative ground/cueも変わるのか、(c)obstacle heightがfusion差分と一致するのかを再検証する。

## 再現コマンドと出力

```bash
python3 src/pm_evaluation/tools/analyze_revisited_terrain_pose.py \
  src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/provenance_1x_retry1/path_samples.csv \
  src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/provenance_1x_retry1/forensic/hazard_cells.csv \
  --output-dir src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/provenance_1x_retry1 \
  --revisit-radius 0.15 --min-time-gap 30
```

出力は`revisited_path_pose_summary.json`と`revisited_path_pose_pairs.csv`。これらは計測診断であって、安全走行可能性の評価ではない。

## 追加調査: ground replacement直前・直後の入力履歴

上の最大z差revisit付近について、同じodomセルを観測した各frameで、更新前のground入力と今回のground入力を記録するdiagnostic replayを追加で実施した。再生時はlocalization launchを1組だけ起動し、終了後にもmapper/playerの残存がないことを確認した。別localizerが重複していた試行は解析に使っていない。

対象ROIの診断CSVは1,982行・62セル分で、fusion mode 2（既存groundを低い候補で置換）は7更新だった。7更新すべてで、旧候補から新候補へのrelative ground変化は`-0.220 m`から`-0.277 m`、中央値`-0.223 m`。これに対し、観測姿勢のbase z変化は`-0.008 m`から`-0.031 m`、中央値`-0.019 m`だった。したがって、この代表的な大きなobstacle cueは「base zが約25 cm飛んだため」では説明できない。更新時のcapture pose zは旧ground候補の撮像時とほぼ同じ範囲だった。

最大の例である絶対セル`(152,124)`（map stamp `1785025121434899091`）は次の通り。

| 項目 | 置換前に採用済みの候補 | 今回採用された候補 |
|---|---:|---:|
| camera pixel `(u,v)` | `(160,276)` | `(224,356)` |
| axial depth | `3.351 m` | `2.011 m` |
| 撮像時base z | `-0.119 m` | `-0.132 m` |
| 撮像時pitch | `-2.333°` | `-0.307°` |
| relative ground | `+0.210 m` | `-0.067 m` |
| relative ground差 |  | `-0.277 m` |

この差`27.7 cm`がlower-ground replaceのobstacle cue`0.290 m`にほぼ対応し、cellはobstacle cueでhazard=1になった。slope`2.85°`、roughness`1.5 cm`、step`5.2 cm`は各閾値未満だった。この例では、旧・新入力のpixel/depthが大きく異なるため、実装ログだけで「深度外れ値」と「同じセルに投影された実際の別面（縁・物体・遮蔽の変化）」を決着させることはできない。pixel差は車両移動に伴う通常の視点変化でも起きるので、それ単独を異常とは見なせない。一方、同じセルの地面として受理された2点のrelative zが27.7 cm違うことは直接確認できた。

今回の追跡で優先順位が上がった原因候補は、z平行移動そのものではなく、(a)深度点のXY投影先が姿勢誤差でずれ別surfaceを同一cellへ混ぜた、(b)道路と縁/物体の境界点がground候補に入った、(c)depth点・intrinsics・camera extrinsic由来の高さ誤差、である。姿勢角は旧候補と今回でpitchが約`2.03°`、yawが約`14.45°`異なるため、poseによる投影XYずれはなお有力だが、現データだけで確定はできない。depthは旧`3.351 m`から新`2.011 m`に変わっており、フレーム中のmin-z点(pixel)をground候補とする方式が縁・別面を拾った可能性も残る。

### 結論の更新

- 「zが急に動いたため、新規地点のabsolute groundだけがずれる」現象は別途観察され、依然としてlocalizationの問題である。
- しかし、今回捉えた黒いobstacle置換7件はbase zの前後差が最大でも約3.1 cmで、relative ground差は22–28 cmだった。この更新群の直接原因をz並進誤差に帰す根拠はない。
- hazard cueの直近の原因は、同じXYセルでgroundとして採用したdepth候補間の大きなrelative-height差。それが真の段差/別surfaceか、投影ずれまたはdepth/ground選択誤りかを次に区別する必要がある。
- 選択ROIだけの調査であり、7月bag全体の全hazardセルに占める原因頻度ではない。

今回の再生出力は`src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/revisit_ground_inputs_1x_clean/`以下。主要証拠は`forensic/hazard_cells.csv`で、上記のmap stampとcell座標を使って再抽出できる。

## 追加調査: 元画像上の画素と姿勢変化

上記の各mode-2更新から旧・新source pixelをbagの画像へ戻した。代表セル`(152,124)`の旧pixel`(160,276)`・新pixel`(224,356)`は、いずれもカラー画像上では舗装路面に見える。目視できる範囲で、該当位置に約28 cmの段差や障害物は見当たらない。ただしpixel位置はmapperがodom上の同一5 cmセルへ投影したという意味であり、実世界でまったく同じ点を見たことの独立な証明ではない。

- 旧depth候補は画像内の有効depth領域の上端近く、路面と遠景の境界に近い。新候補は路面のより下側にある。旧候補が境界付近の不安定なdepthを拾った可能性がある。
- ただし旧frameでこのcellへ入ったsampleは4点で、frame内min/max world-z幅は約`2.39 cm`。新frameは3点で幅約`2.80 cm`。一つだけ孤立した外れ値というより、各frameの小さなsample群全体が違う高さに投影されている。
- 旧・新候補のRGB最寄り画像は各depth stampから約`11.5 ms`差で、両streamとも`640x400`、header frameは`oak_rgb_camera_optical_frame`。したがって画素マーカーを比較するには十分近いが、完全同時刻画像ではない。
- 7月bagには`/oak/depth/camera_info`が記録されておらず、replayは設定済みEEPROM fallback intrinsicsを使用した。intrinsics誤差も評価対象に残る。
- poseのz差が小さいことは再確認できた一方、roll/pitch/yawはsource間で変化している。最新localization構成ではroll/pitchは`/wit/imu`の姿勢を使い、yawは水平EKFから取る。平坦路上でのpitch差・yaw差が実際の車体姿勢/旋回を反映したか、センサ・EKFの誤差かはこの2画像だけでは判定できない。画像でも視点が変化しているため、この姿勢差が明らかな誤りだという証拠もまだない。

したがってオドメトリ誤差は候補に残る。今回弱められたのは「base zの並進だけで説明できる」という仮説であり、pitch/roll/yawやXY推定・時間整合の誤差までは除外していない。また、old sourceはdepth領域の境界に近いので、depth側の候補選択も同じ重さで残る。

### 再現可能な画像抽出

```bash
docker exec --user 1000:1000 --env HOME=/tmp patasmonkey_foxy_dev bash -lc '
  source /opt/ros/foxy/setup.bash
  source /workspaces/ros2_ws/install/setup.bash
  source /workspaces/patasmonkey_ws/install/setup.bash
  python3 /workspaces/patasmonkey_ws/src/pm_evaluation/tools/inspect_ground_source_images.py \
    /workspaces/patasmonkey_ws/bags/rosbag2_2026_07_26-09_17_38 \
    /workspaces/patasmonkey_ws/src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/revisit_ground_inputs_1x_clean/forensic/hazard_cells.csv \
    --output /workspaces/patasmonkey_ws/src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/revisit_ground_inputs_1x_clean/ground_source_images
'
```

出力`ground_source_images/ground_source_contact_sheet.png`は旧・新のdepth/RGB画素を並べた目視用で、対応時刻・画像寸法・姿勢値は同ディレクトリの`ground_source_image_matches.json`にある。RGBマーカーはRGB/depthの画素対応が完全と保証されないため参考とし、depth画像上の画素対応を一次情報とする。

各原因候補を根拠・方法・結果・未確定点ごとに整理した独立レポートは、[7月bag hazard原因仮説検証](20260923_july_hazard_cause_hypothesis_tests.md)を参照。
