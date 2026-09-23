# 7月bag hazard仮説の優先順位付き追加検証

実施日時：2026-09-23 23:58 JST  
対象：`rosbag2_2026_07_26-09_17_38`、既存の1倍速replay/forensic出力、Jetson対象OAK-D S2 EEPROM値  
目的：前回レポートの「次に実施する検証」4項目と、後から整理した4候補を対応付け、現データで判定できる範囲を先に調べる。

## 結論

**この段階で実機bagを取り直す必要はない。** 既存bag由来のsource pixel/depthを使ったオフライン計算と保存済みblack-cell forensic値から、次を確認した。

1. fallbackの`fx, fy, cx, cy`はJetson上の対象カメラEEPROM値と一致済み。さらに、取得したRGB EEPROMの歪み係数Dを使ってsource rayを補正する感度計算を行った。対象とした61,472個のユニークmin-source点では、D補正によるworld-z変化は中央値`0.36 cm`、p90`0.67 cm`、p99`1.08 cm`、最大`3.00 cm`だった。代表cell `(152,124)` の旧・新観測でも、同じcellへ再投影されたmin候補の変化はそれぞれ約`0.47 cm`、`0.62 cm`で、約`27.7 cm`のframe間差をDだけで説明する結果ではない。
2. 既存forensicの黒中心37点のうち30点がraw cell記録へ対応した。対応した30点では、1 frame内の候補点数は中央値6、セル内world-z幅は中央値`1.61 cm`・p90`3.61 cm`だった。ただし最大幅は`25.71 cm`で、全地点が安定していたわけではない。またsupport plane residualは中央値`2.40 cm`、p90`8.33 cm`、最大`48.46 cm`。地面候補の混入や複数時刻の重ね合わせ誤差は依然有力な調査対象。
3. 選定済み代表画像14ファイルは重複を含み、depth stampで数えると6 distinct frameだった。重複を除いた6 frameのzero比率中央値は`7.18%`、画像下部の固定ROI（x=160..479, y=240..399）で0.4–4 mに入るpixel比率は中央値`73.4%`（範囲`67.5–86.5%`）。このROIは厳密に路面だけを切り出したmaskではないため、これを路面depthの有効率やセンサ品質の合否とは解釈しない。

一方、**hazard融合アルゴリズムをmedian/分位点に変えた全bag A/B、および時間方向filter A/Bはまだ実行していない**。現存forensic CSVはセル内sampleのmin/max/count等を記録するが、全raw sample値を保持していないため、保存済みCSVだけから正確なmedian・low percentileを復元できない。これはフルbagを取り直す理由ではなく、次回は同じ7月bagを診断有効にして再生する必要があるという意味である。

## 前回示唆された4検証項目との対応

| 前回レポートの項目 | 今回行ったこと／状態 | 関係する後発候補 |
|---|---|---|
| 1. depth/RGB alignment、無効・飽和値、EEPROM intrinsicsと設定を照合 | Kの4値はJetson EEPROMと一致済み。D補正感度を既存source点で計算。選定14枚のzero/有効depth率を集計。ただしpixel alignmentの実測、7月当時のdriver設定・device identity、depthの飽和状態まではbagから確定できない。**一部検証** | 「intrinsics／Dの感度」「ground候補／depth寄与監査」の入力側部分 |
| 2. 複数RGB-D frameの相対poseを推定し、wheel/VIO/local odomと比較 | 前回の2-frame PnPはinlier率・再投影誤差不足で棄却済み。今回、連続frameの相対poseを新たに確定できる品質のデータは得ていない。**未解決** | 「同一点対応とodom姿勢の独立検査」そのもの |
| 3. min/median/low percentile、support、連続性を比較 | black中心の既存raw forensicでframe内候補数、min-max幅、fusion差、plane residualを横断集計。全raw sampleを持たないためrobust aggregatorの再計算は未実施。**診断の一部を拡張、A/Bは未完** | 「黒セル全体のground候補／depth寄与監査」「融合方式A/B」の空間集約側 |
| 4. データ品質確認後に時間方向filterを比較 | 今回はfilterを導入・比較していない。誤った地面候補や位置合わせ誤差が継続する場合に平滑化が誤高さを強化し得るため、③の後に置く。**未実施** | 「融合方式A/B」の時間方向側 |

## 優先度1：EEPROM Dと投影への影響

### 方法

前回mapperのpinhole ray `(u-cx)/fx, (v-cy)/fy` と、Jetsonから得たRGB EEPROMの先頭8 distortion coefficientsを使って`cv2.undistortPoints`で補正したrayを比較した。入力は`provenance_1x_retry1/forensic/hazard_cells.csv`内の各source min pixel/depth。重複sourceを`source stamp + pixel + depth`で除き、61,472点について、同じ記録済みbase姿勢と既存camera mount仮定を適用してworld XYZ差を求めた。

### 結果

水平位置の補正量はmedianでx`0.13 cm`、y`0.09 cm`、p99はそれぞれ約`2.08 cm`、`2.06 cm`。world-zの絶対差はmedian`0.36 cm`、p90`0.67 cm`、p99`1.08 cm`、最大`3.00 cm`で、5 cmを超えるsampleは0/61,472だった。

代表cellの2画像を、同じ5 cm gridへ再投影して比較すると、旧候補frameのcell sample数はpinhole 4→D補正2、新候補frameは3→2となった。cell内min world-zは旧`-2.89 cm→-2.42 cm`（+0.47 cm）、新`-31.87 cm→-31.25 cm`（+0.62 cm）。補正後も2 frameのcell-min差は約`28.83 cm`残る。

### 判定と限界

取得Dを現在のrational-polynomial camera modelとして解釈した場合、**D補正だけが代表例の約28 cm差を作った、という説明は支持されない**。一方、Dでrayが動く量はセル幅5 cmより小さいとは限らず、cell境界付近では同一sampleのcell所属を変え得る。今回の数値は各source sampleの再投影感度であり、D有効mapperで全mapを作り直したhazard A/Bではない。

また、深度画像がDepthAI pipeline内でどの程度rectify/warpされているか、EEPROM Dが7月当時のpreview/depth rasterに適用すべきcamera modelかは、7月bagだけでは完全に確定できない。現行driverはdepthをRGBへalignしているが、これは7月当時の実行設定が同じだった証明ではない。

## 優先度2：black centerの候補とplane support

### 何を「black center」として調べたか

対象は`provenance_1x_retry1`という1倍速diagnostic replayの`path_samples.csv`で、`center_hazard=100`かつlookahead map timestampがある通過サンプルを抽出した。ここでの「center」は、UGVの現在位置ではなく、**後でUGVが通過する軌跡中心位置に対応するcell**を指す。評価scriptは通過時刻より前のmapのうち、UGVが約2 m先にいる時点のmapを選んでいる。従って「black center」は、実績経路中心に対して、接近前のlocal mapがhazard=1（OccupancyGrid値100、RVizで黒）と評価していたcellである。`100`はdebug表示の上限であり、走行安全の実測ラベルではない。このreplayでは37件が該当した。過去の別replayで数えた41件とは再生・pose差があるため、件数を同一母集団の差分として比較しない。

各path sampleの`x_odom_m/y_odom_m`を5 cm resolutionでfloorして絶対odom cellへ変換し、`lookahead_map_time_ns`、cell-x、cell-yの三つ組でforensic CSVと照合した。これにより「近所の黒セル」を誤って同一点扱いすることを避ける。37件中30件が同じmap timestamp・同じ絶対cellのdiagnostic rowを持ち、7件は見つからなかった。後者は近傍cellへの曖昧な代替matchをせず未照合として残した。異なるreplayでmap更新時刻やposeがずれるため、7件の欠落自体は非hazardを意味しない。

同じmap/cellが複数path sampleから参照されると集計上は複数回数えられることがある。よってここでの30件は完全に独立な30箇所・30試行を意味せず、統計も独立標本の信頼区間ではない。

調べた処理経路は、(a) depth pixelをKで3D化し撮像時刻のcamera→odom poseを適用、(b) 5 cm XY cellごとにframe内min/max/countを集計、(c) frame-minをground candidateとして過去groundと比較し、20 cm以内ならweighted merge、20 cm超低ければground replaceとobstacle evidence追加、20 cm超高ければ棄却、(d) relative-elevation gridの3×3 fresh neighborhoodからplane/slope/residual/stepを計算、(e) slope/roughness/step/obstacleの閾値正規化最大値をhazardへ出し、100に量子化されたcellをRVizで黒表示、という流れである。forensic matchでは(a)〜(e)の全状態を再計算したのではなく、同じmap timestamp・cellに対して、最新sourceの代表pixelとframe内統計、現在ground値、現在のcue/plane-supportを結び直した。過去全frameのraw candidate列がないため、どの過去sampleが最初にcueを作り、その後どう保持・減衰したかまで完全追跡したわけではない。

### frame内candidateをどう調べたか

各matched black centerのdiagnostic rowには、現在map値に対して最新のraw source frameの記録がある。depth imageを`pixel_stride=4`で間引き、0.4–5.0 mの有効depth点をback-projectして5 cm XY cellへ集め、そのcellに入った点の最小world-zを暫定ground candidateとしていた。forensicではそのcellについて、source pixel/depth、同frameのmin/max world-z、sample数、fusion前後ground、過去に採用されたground sourceを保存している。保存されているのは全pixel配列ではなく、min/max代表点と集計値である。

その集計から得た値は次のとおり。

| 観測量（matched black path sample） | 中央値 | p90 | 最大 | 読み方 |
|---|---:|---:|---:|---|
| `frame_sample_count` | 6点 | 9点 | 11点 | このframeで当該5 cm cellに入った有効stride sample数。frame間の累積回数ではない |
| `frame_max_world_z - frame_min_world_z` | 1.61 cm | 3.61 cm | 25.71 cm | 同じframe内・同じcellの点の高さ幅。多くは数cmだが、一部cellでは大きく広がる |
| `ground_after_fusion - ground_before_fusion` | +0.78 cm | +4.13 cm | +56.60 cm | 現在保持中のground値が今回の処理でどれだけ変わったか。正値は上方、負値は下方への変化 |

`ground_before/after`はcell ground stateの更新量であって、必ずしもそのframe candidateのmin/max幅ではない。30 matched sampleの最大`+56.60 cm`を確認すると、cell `(216,412)`の`fusion_mode=1`、すなわち未観測cellの初期化だった。`ground_before=0`は実測groundではなく空cellの初期値なので、これは「既観測の地面が56.6 cm跳ねた」事例ではない。もう1件のmode 1も空cellの初期化（0→-19.85 cm）だった。よって全30件のbefore/after差を既観測cellの時系列変動として読んではいけない。

weighted mergeのmode 3（25件）に限ると、`ground_after - ground_before`は中央値`+1.00 cm`、p90`+4.13 cm`、最大`+5.13 cm`（絶対差では中央値`1.42 cm`、p90`5.13 cm`、最大`10.86 cm`）だった。mode 2は1件あり、cell `(152,124)`で`-28.88 cm`のlower-replaceが起き、旧ground`-2.89 cm`から新frame-min`-31.78 cm`へ下がった。mode 0は2件で更新量ゼロだった。このblack時点の最新source snapshotでは、通常のweighted mergeは小さめだが、十数〜数十cm級のtailとして少なくともこのlower-replace例がある。なお、mode counts（`0:2, 1:2, 2:1, 3:25`）は最新source処理の分類で、過去に蓄積されたobstacle evidenceの全生成履歴ではない。

### plane supportとcue値をどう調べたか

terrain feature計算はrelative elevationのうちageが3秒以内のfresh cellを対象に、半径1 cellの近傍（最大3×3）で局所平面`z=ax+by+c`をfitする。最低5 neighborが必要である。forensic support CSVはmatched black target cellごとに、このfitへ使われたfresh neighbor cellを出し、各neighborのrelative elevationからtarget cellのfit平面予測値を引いた残差を記録する。今回266 support rowsを得た。理論上は30 target×最大9 neighborに近い件数だが、unknown/stale cellやmap端でsupportが減る。隣接target間で同じneighborが再利用されるため、266行は266個の独立測量ではない。

この266行の`abs(plane_residual_m)`は中央値2.40 cm、p90 8.33 cm、最大48.46 cmだった。残差は「実地面の凹凸」そのものではなく、fit平面と地図cell値の不一致である。高い残差は、実際のsurface変化のほか、ground candidate混入、depth誤差、違う時刻のposeで観測した点のずれ、低いsupport等でも生じる。また、このp90はcenterのroughness値そのものではない。roughness cueは各target neighborhood内の残差RMSで、`0.03 m`以上が閾値超過となる。

黒閾値を超えていたcueは重複を許して数えた。30 matched path sample中slopeは20件（20°以上）、roughnessは17件（平面残差RMS 3 cm以上）、stepは値が定義された28件中21件（support付き残差peak-to-peak 7 cm以上）、obstacleは4件（暫定obstacle-height 20 cm以上）。cue組合せは`slopeのみ 7`、`slope+roughness+step 12`、`stepのみ 3`、`roughness+step 3`、`slope+step 1`、`roughness+step+obstacle 2`、`obstacleのみ 2`。合計30件で、単一要因に分けられない。これは地面の実測傾斜や段差を確認した結果ではなく、現在のmapから計算されたcueが閾値を越えた数である。

### この検証から導ける結論

多くのmatched black centerでは、最新1 frame内で同じcellに入った点のz幅は数cmだった一方、support plane residualには大きなtailがあり、少数には同frame内z幅も大きいcellがあった。ground updateの大きな`+56.60 cm`は初期化であって再観測による跳ねではない。実際に既存値からのlarge lower-replaceとして記録されたのはこのsample setでは1件（`-28.88 cm`）だった。したがって「多数のセルで何十cmの更新が繰り返された」とは言えないが、「大きなtailが全くない」とも言えない。さらに、同じcell番号が別frameで同じ実地点を表すかはodomとcamera poseの正しさに依存し、今回のcell一致だけでは証明できない。

また、この集計はsaved latest-source min/maxと周辺grid値の監査であって、全sampleのraw pixel/depth列を黒cell全てについて保存した解析ではない。中央値・分位点でground候補を選ぶ再計算や、各cueに伝播したpixelをすべて逆追跡するには、同じ7月bagをoffline forensic modeで再生し、candidate distributionを追加保存する必要がある。今回の値だけから「depthが悪い」「odomが悪い」のどちらかへ断定しない。

## 優先度3：選定RGB-D画像のdepth品質を再確認

7ケースから保存されたold/new depth image 14ファイル（640×400, uint16 mm）を読み、depth stampで重複を除いた6 frameについてzero率と固定下部ROIの有効範囲を集計した。zero率は`6.68–7.49%`、下部ROIの0.4–4.0 m valid率は`67.5–86.5%`。

これは明らかな全画面depth欠落ではないことを示すが、画素のRGB対応精度、道路surfaceだけのvalid率、ステレオ誤差、invalidが何の原因で生じたかは確定しない。過去に代表pixel近傍81点がすべてvalidでdepth中央値もsource pixelと一致した結果とも整合し、「代表sourceがisolated zero/spikeだった」説明は弱い。ただしdepth系統誤差・境界誤差までは除外しない。

## 今回まだ行っていないもの

### 複数frame相対poseによるodom検証

既存visual pose結果は、対象例でfeature/depth対応が不足しquality gateに落ちている。今回使える保存画像もその代表frame群で、新しい独立poseを支持する材料は増えていない。過去のPnP値を弱い閾値で採用することはしない。追加で行うなら、bagから静止建物等のtextureがある連続frameを抽出し、複数frame束ねた推定とinlier/reprojection基準を事前定義した上で実施する。

### median/percentile fusionとtemporal filterのA/B

保存済み`hazard_cells.csv`はframe内min/maxとcountを残すが、同一cellへ入った各sampleの全z値・raw pixel集合は保持しない。ゆえにmedian/low percentileを後処理で捏造して比較することはできない。次の正確なA/Bでは、7月bagを再生し、diagnostic-only modeでcellごとの候補配列（または十分な分位点計算用統計）を保存する必要がある。Jetson通常処理にCSV出力を持ち込む必要はない。

時間方向filterもこのA/Bに続けて同bagの同一cue/thresholdで比較する。遅延、真の段差保持、false positive/negativeを別指標で見る。hazard black率だけの低下は成功判定にしない。

## 推奨する次の順序

1. **同じ7月bagをoffline forensic設定で再生し、全center-black sampleのper-pixel/per-cell provenanceとcandidate分布を保存**する。outputはblack中心と狭いROIに限定し、通常runtimeではoffのままにする。
2. 保存候補に対し、pinhole/D補正、cell min/low percentile/median、およびfresh support gateを比較する。Dの単点感度は大きなz段差原因ではなさそうだが、cell再割当てとhazard map差を全体A/Bで確認する。
3. 色・depthが特徴を持つ時間連続区間を選んでmulti-frame relative poseを試し、品質条件を通る場合だけ独立odom比較を行う。成立しなければ「このbagでは姿勢真値を判定できない」と記録し、そこで初めて新規実機bagに静止構造物の視野・CameraInfo・device diagnosticsを含める計画を立てる。
4. その後にmedian/temporal filterを同じbagで評価する。

## 再現に使った既存出力・設定

- black-center forensic集計：`src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/provenance_1x_retry1/forensic_summary.json`
- 元cell CSV：同ディレクトリの`forensic/hazard_cells.csv`
- black中心とsupport：`matched_center_black.csv`、`matched_center_black_support.csv`
- representative depth image：`revisit_ground_inputs_1x_clean/ground_source_images/case*_depth_mm.png`
- source pixel・pose・画像timestamp：`revisit_ground_inputs_1x_clean/ground_source_images/ground_source_image_matches.json`
- 対象camera intrinsics fallback：`src/pm_perception/config/depth_elevation_mapper.yaml`
- mapperのpinhole projection：`src/pm_perception/pm_perception/depth_projection.py`
- 現行driverのEEPROM CameraInfo生成：`/home/kazuho/ros2_ws/src/depthai_driver/depthai_driver/oakd_vio_rgbd_node.py`
- 前回の仮説・方法・未解決事項：`notes/reports/20260923_july_hazard_cause_hypothesis_tests.md`

今回の感度計算・depth画像統計は既存のCSV/PNGに対する読み取り専用の一時解析で、production mapperやlocalizationを変更していない。完全なD有効全bag map再生成、およびaggregation/filter A/Bは未実施である。

## 今回行った検証の詳しい説明

### 1. カメラ内部パラメータKと歪み係数Dの意味

今回の投影検証に関係するカメラモデルは、内部行列`K`と歪み係数列`D`で表される。

| 記号 | 今回使った値 | 意味 |
|---|---:|---|
| `fx`, `fy` | 574.288269, 574.288269 pixel | 水平・垂直方向の焦点距離。pixel座標の差をカメラ座標上の角度／距離へ換算する尺度 |
| `cx`, `cy` | 354.750854, 215.232620 pixel | 光軸が画像面と交差する主点。pixel座標の原点をどこに置くかを決める |
| `D` | `[10.22323513, -109.25149536, -0.00044202, 0.00103162, 320.91735840, 9.92284966, -107.20381165, 315.32150269]` | EEPROMのRGB camera distortion係数。driverはCameraInfoの`rational_polynomial`形式として先頭8個をpublishする |

Kだけのpinhole投影では、画像pixel `(u,v)` とdepth `Z`からカメラ座標を次のように作る。

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth
```

このmapperは実際にこのKのみの計算をしており、CameraInfoのDを使っていない。D補正側では、観測pixelをレンズ歪みのない理想画像上のrayへ逆変換してから同じZを掛けた。最初の4係数は通常のrational-polynomial表現における半径方向・接線方向歪み、後半は半径方向分母項に対応する。係数値をそのまま距離や誤差cmとして読むものではない。

### 2. D補正の数値をどう計算し、何を意味するか

対象は`provenance_1x_retry1/forensic/hazard_cells.csv`にある各cellのframe-min source pixelとaxial depthである。同一stamp・pixel・depthを重複排除し、61,472個のsource rayを得た。各rayを「既存Kだけ」と「EEPROM Dでundistort後」の2通りに変換し、同じdepth、同じ記録済みcapture pose、同じcamera mount仮定を使ってworldへ移した。比較量は2方式の位置差であり、Dを有効にした実hazard mapの再生成ではない。

結果のworld-z差中央値`0.36 cm`、p99`1.08 cm`、最大`3.00 cm`は「Dが正しいなら全てのheight errorがこの範囲」という保証ではない。あくまで選択済みsource ray群と指定したモデル／TFでの差である。また水平位置はcell resolution`0.05 m = 5 cm`に対しp99約2 cm、最大は約4 cm動き得る。したがって、5 cm cellの境界近くではcellへの割当てが変わる可能性がある。

代表例`(152,124)`では旧観測と新観測のmin候補がそれぞれD補正後も同じcellに残った範囲で、z変化は約0.47 cm、0.62 cmだった。frame間のmin-z差はpinhole時の約28.98 cmからD補正後も約28.83 cm残った。この例に限れば、Dは段差状差分を数mm変えるが、約28 cm差の直接原因には見えない。

### 3. 5 cm cell内の候補分布・hazard cue集計の読み方

mapper設定の主要値は次のとおり。

| Parameter | 値 | この検証に関係する意味 |
|---|---:|---|
| `resolution` | 0.05 m | XY gridの1セル幅。別々の実世界点も同じ5 cm cellへ入ることがある |
| `pixel_stride` | 4 | 深度画像の縦横を4 pixelおきに処理する設定。理想的には全pixelの約1/16を点化する |
| `min_depth`, `max_depth` | 0.4 m, 5.0 m | 投影へ使うdepth範囲。0やこの範囲外はmapperに入らない |
| `ground_merge_threshold` | 0.20 m | 新しいframe-min候補が旧groundより20 cm超低ければgroundを置換し、差をweak obstacle evidenceへ記録する。旧groundとの差が±20 cm以内なら分散重み付きで融合し、20 cm超高い候補はgroundへ採用しない |
| `feature_max_observation_age` | 3.0 s | terrain cueの局所平面へ使うcellをfreshとみなす最大経過時間 |
| `feature_min_neighbors` | 5 | 局所平面に必要な最低support数 |
| `hazard_slope_limit_deg` | 20° | slope cueの正規化上限／黒判定閾値 |
| `hazard_roughness_limit` | 0.03 m | 平面残差RMS 3 cmの閾値 |
| `hazard_step_limit` | 0.07 m | support付き残差peak-to-peak 7 cmの閾値 |
| `hazard_obstacle_height_limit` | 0.20 m | 暫定obstacle-height cue 20 cmの閾値 |

`frame_sample_count`は、あるdepth frame内で5 cm cellへ投影されたpixel-stride後の有効点数であり、複数frameにまたがる累積観測回数ではない。`frame_min_world_z_m`と`frame_max_world_z_m`の差は同一frame内でそのcellに入った点の上下幅。今回その幅のmedianは1.61 cm、p90は3.61 cmだが最大25.71 cmであり、「各セル・各frameの候補群はいつも狭い」とは言えない。

`ground_before_fusion_m`と`ground_after_fusion_m`は、そのframeの候補を融合する前後で保持ground値がどう変わったかを示す。今回の`ground_after - ground_before`は中央値+0.78 cm、最大+56.60 cmだった。正値はground値が上がった更新、負値は下がった更新を表す。これら全てがobstacle cueになったわけではなく、どのfusion分岐だったかは`fusion_mode`で区別される。少数の大きな更新がtailにあることを示すため、単純平均で改善すると結論する前に、各候補をraw depth/pixelと結びつける必要がある。

この実装での`fusion_mode`は`0=既存より20 cm超高く不採用`、`1=空セルへ初期登録`、`2=既存より20 cm超低くgroundをreplace`、`3=±20 cm以内で重み付きmerge`。mode 2では`旧ground - 新candidate`をobstacle-height evidenceとして残す。したがって高い側へ変わった大きなground差と、black hazardを作る低いground replace由来のobstacle cueは区別して読む必要がある。

`plane_residual_m`は、support pointの高さとlocal fitted planeとの差である。今回の266 support行の`abs(residual)`は中央値2.40 cm、p90 8.33 cm、最大48.46 cmだった。これは「地面の真の凹凸」と同義ではない。誤ったground候補、depth誤差、別時刻poseでずれたpointのいずれも残差を大きくする。

30 matched black centerのcue閾値超過数は重複を許す。slope 20/30はslopeが20°以上、roughness 17/30は平面残差RMSが3 cm以上、step 21/28はstep値が定義された28地点中7 cm以上、obstacle 4/30はobstacle cueが20 cm以上だったという意味である。これは「各cueの唯一原因割合」でも「地面が実際にその角度・段差だった割合」でもない。hazardは複数cueの最大正規化値なので、一地点で複数条件を超える。

### 4. depth画像のzero率・有効率の定義

対象画像はsource forensicに保存されていたold/new画像14ファイルで、depth header stampで重複を除くと6 frame。画像は`16UC1`相当のmm値で読む。zero率は`depth == 0`の全640×400 pixel比率。ROIは`x=160..479, y=240..399`を固定切り出し、そのうち`400 <= depth_mm <= 4000`の比率を「0.4–4 m valid率」とした。

ROIは路面segmentation結果ではない。路面以外、遮蔽物、画像境界も含み得るため、`73.4%`中央値を「路面の73.4%が測れた」と解釈してはいけない。またzero以外のdepth誤差はこの比率に反映されない。これらは、選択frame全体がほぼ欠測だったわけではないことを確かめる粗い入力健全性checkであり、Camera calibration・pixel alignment・stereo精度の合否判定ではない。

### 5. 今回の結果から言える範囲

- K fallbackが実機EEPROMと一致することと、D補正のsource-ray感度が代表例の約28 cm差より小さいことは確認できた。よって「fx/fy/cx/cyの値違い」や「D補正だけ」が今回の代表差を作った説明は弱い。
- 選定frameのdepthには相当数の有効値があり、代表点の81-pixel近傍にもvalid depthがあった。ただし、同じ実空間面を別frameが見ていたと確認できてはいない。
- 5 cm cellに入るframe内点群の大半はcm単位の高さ幅だが、広い幅のtail、ground updateの大きなtail、plane residualの大きなtailも見つかった。これらはcandidate contaminationまたはpose重ね合わせ異常の兆候になり得るが、原因を一意に特定しない。
- 黒cell raw候補を全て保存したA/Bではないため、median/percentile、D込みのhazard分布、temporal filterの有効性はまだ不明。ここは追加forensic bag replayが必要。
