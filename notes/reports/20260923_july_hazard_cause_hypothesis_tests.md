# 7月bagのterrain hazard原因仮説と検証記録

## 目的と範囲

平坦な舗装路に見える走行でhazard=1が発生した原因を、仮説ごとに切り分ける。特に代表セル`(152,124)`で観測された、ground候補間のrelative elevation差約`27.7 cm`を対象にする。

使用データは`rosbag2_2026_07_26-09_17_38`を、現行の分離localization launchとdepth terrain mapperで1倍速replayしたもの。対象セル周辺だけをforensic記録した`revisit_ground_inputs_1x_clean`を使った。結論はこのセル・小ROIについてのものとし、7月bag全hazardへの原因割合とは解釈しない。

重要な定義：ここでいう「同じセル」はodom poseとカメラTFで投影した点が同じ`5 cm × 5 cm` grid cellに入ったことを指す。地面上の実同一点を外部測量などで確認した意味ではない。odom誤差そのものをこの対応付けで判定すると循環論法になり得る。

## 代表例の再確認

代表cellの旧候補はpixel`(160,276)`、axial depth`3.351 m`、odom上のworld z`-0.0289 m`、relative elevation`+0.2100 m`。約1秒後の新候補はpixel`(224,356)`、depth`2.011 m`、world z`-0.3187 m`、relative elevation`-0.0665 m`。その差は`-0.2766 m`で、ground lower-replace時のobstacle cueは`0.2898 m`だった。

新候補はslope`2.85°`、roughness`1.48 cm`、step`5.15 cm`で、これらの閾値によるhazardではない。hazard=1の主因は置換差分に由来するobstacle cueである。

カラー画像上では旧・新pixelとも舗装路面に見え、該当箇所に28 cm級の障害物は目視できない。ただし両画素は同じ視点・同時刻ではない。

## 仮説別の検証

### H1: 1個の孤立したdepth outlierがground差を作った

**根拠**：ground candidateは各cell内の低い点から選ばれる。ステレオdepthは画素・距離・テクスチャ境界によって誤差が出る可能性があり、旧点は有効depth領域の上端付近にあった。

**検証方法**：元bagのdepth frameで各source pixelの`±16 px`範囲を4 px間隔で集計し、同一frame内の当該cellへ入った全sampleのworld-z幅も比較した。

**結果**：旧pixelのdepthは`3.351 m`で、周辺81サンプルの中央値も`3.351 m`、valid率は81/81だった。新pixelは`2.011 m`で、周辺81サンプルの中央値も`2.011 m`、valid率も81/81だった。各frameの同じcell内world-z幅は旧`2.39 cm`、新`2.80 cm`。

**判定**：この例を単独の塩胡椒状spikeで説明する仮説は弱まった。旧pixelがdepth有効領域の境界付近であること、広い近傍窓では距離勾配があること、センサの系統誤差や視差境界誤差までは否定しない。

### H2: 「セル内最小zをgroundにする」選択だけで約28 cm差になった

**根拠**：mapperは1 frame内でcellに入った点の最小zを暫定ground候補とする。低い候補は以前のgroundをreplaceし、その高さ差をobstacle evidenceとして残す。

**検証方法**：代表cellの各frame内min/max zと、旧・新source pixel/depthを比較した。

**結果**：minとmaxの幅は旧`2.39 cm`、新`2.80 cm`。旧・新frame間のrelative ground差`27.66 cm`より一桁小さい。

**判定**：frame内minとmedian/maxの選び方だけでは原因を説明できない。一方、「平面外の別surfaceや道路端をground候補にした」可能性は残る。これは単なるmin選択とは別の仮説である。

### H3: 実際に同一平坦路面を2回観測したが、z並進推定誤差が差を作った

**根拠**：平坦面がodom zの変化に応じて別高度へ投影される可能性がある。別地点のabsolute elevationがずれると、融合時に段差状cueを作り得る。

**検証方法**：採用済み旧・新sourceの撮像時base poseを比較した。

**結果**：base zは`-0.1186 m`から`-0.1320 m`へ約`-1.34 cm`変化しただけ。relative ground差は`-27.66 cm`。この更新にz並進の急変は見られない。

**判定**：少なくとも代表例では、z並進単独を直接原因とする説明に合わない。z推定の全行程上の異常を否定するものではない。

### H4: IMU/TFの時刻ずれで、古いroll/pitchをdepth画像へ適用した

**根拠**：現行localizationはroll/pitchをWIT IMUから取り、mapperはdepth画像timestampのTFを要求する。姿勢の合成・転送時刻がずれると、点群の上下方向投影が変わり得る。

**検証方法**：各depth source stampに最も近いbag内`/wit/imu` header stampと姿勢を照合し、mapperのcapture roll/pitchと比較した。またreplayのTF dropログを確認した。

**結果**：旧候補は最近傍WIT stampとの差`0.85 ms`、roll差`0.003°`、pitch差`0.015°`。新候補は`2.47 ms`、roll差`0.018°`、pitch差`0.065°`。この区間のTF dropは`0`。旧・新capture pitch差は約`2.03°`。

**判定**：最近傍WITデータの取り違えや大きな時間遅れという仮説は支持されない。WIT姿勢に共通のbias、IMU取付角誤差、pitch自体の真値との差は未検証。yawはWITではなく水平EKF由来である。

### H5: 位置・yaw・姿勢を含むodom誤差で、同じcellに別位置のsurfaceが重なった

**根拠**：旧・新candidateのcapture yawは`65.31°`から`79.76°`へ変化し、base XYも移動している。同じodom cellという判定はこのposeに依存する。RGB画像にも視点変化がある。

**検証方法**：旧・新RGBのORB特徴とdepthから相対カメラ姿勢をPnPで推定し、odom姿勢差と比較した。さらにodom/URDF poseで各source pointを相手frameへ逆投影し、相手depth画像の値を調べた。

**結果**：PnPは31個のdepth付きfeature matchのうち8個しかRANSAC inlierにならず、inlier率`25.8%`、refine後の中央値再投影誤差`28.8 px`だったためquality gateで棄却した。したがってPnPからodom誤差の有無は判定できない。

旧source pointをodom poseで新frameへ投影するとpixel`(226.6,272.9)`、予測depth`2.02 m`。新depth画像の同位置と周囲3×3は`0.4–5 m`の有効depthなし（中心値`6.032 m`）。逆に新source pointを旧frameへ投影すると`(157.3,326.3)`、予測`3.33 m`だが、旧depthは周囲3×3全て約`1.885 m`だった。

**判定**：実世界の同一地点をdepthで相互確認できず、「同じcellだから同じ路面点を見た」とは言えないことが具体的に確認された。しかし、この不一致だけでodom誤差とは断定できない。視線遮蔽、depth無効、候補が実際には別面、intrinsics/extrinsics誤差、odom姿勢誤差のいずれでも起き得る。visual PnPが失敗したため、odomの妥当性は未解決。

### H6: intrinsics / camera optical extrinsicの誤差

**根拠**：back-projectionとTF変換はintrinsics、RGB/depthのpixel alignment、URDFのcamera mountに依存する。旧bagではdepth header frame名をURDF optical frameへoverrideしている。

**検証方法**：bag metadataでcamera_infoの有無を確認し、使用parameterとheader frame・画像寸法を確認した。

**結果**：7月bagに`/oak/depth/camera_info`メッセージはなく、mapperはfallback`fx=fy=574.288`、`cx=354.751`、`cy=215.233`（640×400）を使用。RGBとdepthは同寸法でheader frameは`oak_rgb_camera_optical_frame`。URDF側のcamera optical frameは`rgb_camera_optical_frame`で、旧bag replayでは同じ位置・向きと仮定してoverrideしている。

**判定**：この仮定・fallback値の現物校正は未検証。camera info不在は確認できたが、intrinsics/extrinsicsが今回の差をどれだけ作ったかはまだ数値化していない。depth画像を左右画像から再計算するのではなく、まずOAK-Dの公称出力と設定・校正値の照合でよい。

### H7: 全セル平均化すればhazardを抑えられる

**根拠**：独立した一時的誤差を多数frame融合で薄められる可能性がある。

**検証方法**：今回は平均化方式のA/B replayは行っていない。まず旧・新候補の分布とsource consistencyを検証した。

**結果**：代表例は単一pixel spikeではなく、各frameの局所候補群がそれぞれ狭い高さ幅を持つ一方、frame間の相対高さが大きく異なる。また採用前後のsource pointが実同一点か確認できない。

**判定**：平均化が有効という仮説は未検証。複数frameで同じ系統誤差や誤投影が続く場合、単純平均は誤高さへ引き寄せられ、本物の段差も平滑化する。原因評価とは分けて、後続で中央値/分位点、支持数・時間持続性gate、multi-hypothesis保持などを同一bagで比較する。

## 今回の総合結論

1. hazard cueが発生した計算上の直接経路は確認できた：ground candidateのlower replace差分`約29 cm`がobstacle cueになった。
2. 旧・新候補は各frame内で安定した小点群であり、単発depth spikeやframe内min選択だけを主因とする説明は弱い。
3. 旧・新pointのz推定差は大きいが、base z並進差は約1.3 cm。z並進単独では説明しにくい。
4. 最近傍WIT姿勢とTF姿勢は時刻・数値ともよく一致。stale attitudeではなさそうだが、WIT姿勢の真値性は未確認。
5. 同じ5 cm odom cellが実同一点を示すことは確認できず、RGB-D独立相対姿勢推定も品質不足で不成立。odom誤差、surface/candidateの違い、camera calibration、depth valid-region/occlusionは未確定のまま残る。

以上から、現時点で「terrainは平坦なのにlocalizationが必ず悪い」または「OAK-D depthが悪い」のどちらかに断定する証拠はない。確認できたのは mapper上のheight hypothesis間の大きな差と、独立した同一点対応付けができていないこと。

## 次に実施する検証

1. OAK-D depthとRGBのalignment、depth invalid/saturation値、使用中のEEPROM intrinsicsを、bagに記録されたdriver設定・カメラID・現行driverパラメータと照合する。まずセンサ側設定だけを対象にし、stereo再構成方式自体は変更しない。
2. localizationのXY/yaw姿勢を独立に見るため、地面だけでなく静止壁・建物を含めた複数frameのRGB-D相対pose推定を改善する。特徴数/inlier率/reprojection errorが基準を満たす区間に限定し、wheel/VIO/local odomと比較する。今回のPnP失敗結果を再利用してposeを断定しない。
3. 各セルへminだけでなくmin/median/low percentile、frame内support、近傍面との連続性を記録し、同一セル置換がどの集約値で発生するかをA/B評価する。
4. 有効データが取れた後にのみ時間方向filterを比較する。遅延、真段差の保持、hazard false positive/negativeを個別に評価し、単なるblack率低下を改善とみなさない。

## 再現・関連出力

- Replay summary: `src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/revisit_ground_inputs_1x_clean/summary.json`
- pixel provenance: `.../revisit_ground_inputs_1x_clean/forensic/hazard_cells.csv`
- RGB/depth画像と近傍depth統計: `.../revisit_ground_inputs_1x_clean/ground_source_images/ground_source_image_matches.json`
- contact sheet: `.../revisit_ground_inputs_1x_clean/ground_source_images/ground_source_contact_sheet.png`
- visual pose試行: `.../revisit_ground_inputs_1x_clean/ground_source_images/visual_rotation_comparison.json`
- 画像抽出script: `src/pm_evaluation/tools/inspect_ground_source_images.py`
- RGB-D回転比較script: `src/pm_evaluation/tools/compare_ground_source_visual_rotation.py`

`compare_ground_source_visual_rotation.py`のPnPは本区間でquality gateに落ちる。閾値を下げて出た回転値をpose evidenceとして採用しないこと。
