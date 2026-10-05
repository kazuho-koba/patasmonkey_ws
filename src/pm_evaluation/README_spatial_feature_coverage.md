# 前方半径2 m・無期限の空間保持によるoffline coverage評価

秒ベースの`evaluate_frame_feature_coverage.py`を上書きせず、別実行scriptを追加した。
通常perception、YAML、launchは変更しない。MCAPと保存mapper traceを直接読み、ROS nodeや
センサ・モータを起動しない。tools追加なのでbuild不要。

## 実行（Foxyコンテナ）

```bash
cd /workspaces/patasmonkey_ws
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python3 src/pm_evaluation/tools/evaluate_spatial_feature_coverage.py \
  bags/rosbag2_2026_07_26-09_17_38 \
  --mapper-trace notes/reports/perception/20261003_100000_depth融合_stride比較_assets/stride1/trace/mapper_trace.jsonl \
  --output /tmp/new_spatial_coverage \
  --radius 2.0 --stride 1 --single-frame-obstacle
```

出力先は存在しない新規directoryを指定する。別bagにはそのbagのmapper traceが必要。
半径5 mを比較する場合は`--projection-map-size 12`等で投影領域を確保し、
半径2 m側にも同じ値を指定する。trace TF・セル幅は変えず、offlineの一時gridだけを広げる。
depth上限も変更する場合は`--max-depth 10`等を指定する。半径だけではdepth filterは
変更されない。半径2／5／10 mにdepth上限も揃える比較では`--max-depth`へ同じ値を
渡し、投影領域を全条件`--projection-map-size 22`で統一する。深度はoptical-z[m]で、
地上水平距離と厳密には一致しない。bagに元から存在しない／無効な深度は復元しない。
traceの設定・intrinsics・camera→odom・base poseを再利用し、採用stamp・画像寸法・frameを
確認する。最新オドメトリはtrace採取時に再計算済みで、ここでは再計算しない。

## 領域・保持・評価

- 観測・格納対象は自車XY中心から半径`--radius`m以内、yaw前方の半円。cell中心で判定する。
- 独立画像全体でfeatureを計算するため、境界セルのsupportに半円外の同じ画像の点を使える。
  その結果のうち半円内のセルだけを保持する。未観測セルを外挿・補間しない。
- 地形3cueは同じframeで全て有効な組だけ更新する。`--cue-wise`でcue別保持へ切替可能。
- obstacleは独立channel。`--single-frame-obstacle`でconfidence待ちを省く。
  既定では2画素以上の実測点があり、max−minが`obstacle_min_height`（今回0.03 m）
  未満ならobstacleを0に更新する。他cueの黒は解除しない。未観測・1画素・map外は
  解除せず、閾値ちょうども解除しない。`--obstacle-clear-min-pixels`で画素数を2以上へ
  変更できる。`--legacy-obstacle-retention`で旧方式を再現する。
  解除条件はconfidence待ちとは独立で、単一frameオプションなしでも適用される。
  方式・画素数・閾値はsummaryの`obstacle_update`へ記録する。
- 時間期限・通過後距離・rolling windowからの退出による破棄は行わない。今回の「制限なし」は
  訪れた空間の観測済みセルを無期限に蓄積する意味。長距離bagではmemoryは増える。
- 各frameで前方半円全体の独立／保持coverageを出す。未観測の近接・FOV外部分も分母に含む。
- 通過経路はtraceのbase poseから`--path-spacing`(既定0.25 m)で抽出する。
  `first_front_roi_entry`は将来通過する中心位置が初めて半円内に入った時のfootprint評価。
  一定の経路距離2 m手前ではなく、直線距離が2 m以下かつ前方に入った時点である。
  最初から範囲内、旋回等で側面から入るケースも含む。footprintの一部が半円外ならunknownも残る。
- `before_passage`は通過stampと同stampの画像を入れる前の保持地図を評価する。
  未来の経路は対象位置のラベルにしか使わず、未来のdepthは使用しない。

## 出力

保持期限だけを同条件で比較するには`--compare-age-seconds 3`を加える。
無期限storeや既定結果は変えず、同じ通過時点・同じ観測履歴へ3秒期限を付けた
`path_age_reference`と`age_reference_footprints.csv`を追加する診断である。

- `summary.json`：条件、半円内frame重み統計、経路距離sampleの2イベント統計。
- `front_roi.csv`：各frameの前方半円の独立／保持coverage・黒数。
- `path_footprints.csv`：半円への初回進入時と通過前のfootprint評価。
  各cueの`*_black_cells`も保存し、同じfootprint内の原因重複を確認できる。
- `summary.json`の`path_causes`：原因別の関与footprint数（重複あり）、
  排他的な原因組合せ、cue別の延べ黒セル数。黒判定は主評価と同じ100への丸め。
- `retained_features.npz`：絶対odom cell、metricな4cue、cueごとの観測stamp・support、resolution。

hazard既知・地形3cue完全・4cue完全を分ける。黒率は1セル以上knownのfootprint/ROIを分母
にする。ROI統計はframeごとの半円領域であり、経路footprint率と混同しない。

NPZのcue stampから元traceのframe pose・TFへ対応できる。supportはplane近傍数、stepの
前後supportの小さい方、obstacleのpixel数。pixel数は独立観測confidenceを保証しない。
将来のtime/freshness/pose訂正判定用に`SpatialFeatureStore.eligible`を分離した。

## 近距離観測優先（既定ON）

有効な観測が既に保持されていて、新観測の水平距離が最後に受理した観測より
0.05 m超遠く、かつ旧観測の点数が新観測以上の場合だけ更新を拒否する。
terrain三指標は同一frame組で保持し、点数は有効elevationの3×3patch合計。
obstacleは対象cell点数。実際の近傍幅はfeature_neighborhood_radius_cellsに従う。

**まだ近傍観測がなくても、遠方の初回観測は受理する。** 絶対距離による拒否ではない。
旧観測に点数不足がある場合は遠方の新観測を受理し、近い観測・同程度距離の新観測も
受理する。unknownで過去値を消さず、拒否時はstamp・距離・点数も更新しない。
obstacle解除にも同じ条件が適用され、過去の危険が残りやすくなる可能性がある。

- `--near-distance-margin 0.05`：遠距離化と判定する水平距離差[m]。
- `--latest-observation-wins`：比較用に近距離優先をOFFにし、従来の最新有効値更新へ戻す。
- `summary.json`の`near_observation_priority`：実効設定と指標別拒否更新数。

適用先はこの**独立frame指標を空間保持するオフライン評価**。通常mapperの高さ融合や
ROS launchの既定は変更しない。品質診断・方式E診断は旧baselineを維持するため、
それぞれの専用入口では距離優先を無効にしている。既存の比較レポートも変更しない。
現在はfuture禁止だけで、未実装の動的物体解除・pose補正を実施したと見なさない。

## 制約

高さ融合、z補正、継ぎ目の幾何検証、E/F、補間、完成版の危険証拠融合は行っていない。
無期限保持は静的地形のcoverage検証用で、動的物体・自己位置ドリフト・再走行方向の違いを
無視して実機走行へ採用する仕様ではない。観測stampとsupportを残すのは今後の拡張用。
撮像stamp基準であり、callback・評価完了遅延を反映した停止余裕の試験ではない。
低い高さ幅は安全なfree-spaceの保証ではなく、障害物上面だけの観測でも成立しうる。
通常perceptionの更新規則は変更しない。旧診断scriptは同じmainへの互換入口として残す。
過去レポート・保存済み評価結果は書き換えない。
