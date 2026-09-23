# 7月bag：実走行経路上のterrain hazard評価

評価日：2026-09-23

## 問いと結論

人間が遠隔操縦で実際に通った経路が、通過前の地形地図ではどう判定されたかを調べた。
車体中心が2 m手前で観測できていた325地点のうち、37地点（**11.4%**）はhazard=100
（RVizで黒）だった。幅0.45 m・長さ0.55 mの暫定車体占有範囲で見ると、観測のある346地点中
207地点（**59.8%**）に少なくとも1個の黒セルがあった。黒セルが占有範囲の25%以上を
占めたのは64地点（**18.5%**）である。

したがって、全地図で約32%が黒という値だけでは走行可能性を評価できないが、**実績として
通過した地点にも黒判定が相当数存在する**。現在のhazardをそのままNav2の通行禁止costに
変換することは妥当ではない。特に車体占有範囲内の孤立した黒セルをどう扱うかと、中心経路上
に連続して現れる黒判定の原因を検証する必要がある。

## 入力と評価方法

- bag: `bags/rosbag2_2026_07_26-09_17_38`、1倍速で全区間再生。
- bag内の古い`/tf`・`/odometry/local`は再生せず、現在の`separated_offroad`相当の
  localizationで`odom -> base_link`と`/odometry/local`を再計算。
- 現行`pm_perception`のStage 3.1で`/depth_elevation_mapper/terrain_hazard_debug`と
  4 component mapを生成。blackはOccupancyGrid値100、unknownは-1。
- 再計算odomの軌跡から約0.25 mごとに通過地点を抽出。各地点に対し、車体がその地点から
  **2.0 ± 0.25 m手前**にいたときにpublish済みの地図を採用。前進方向の候補だけを使用。
  地図時刻より後のodometryは位置決定へ使わない。時間差は0.97〜11.86秒、中央値2.09秒。
- 車体中心のgrid cellと、通過姿勢での暫定長方形footprint（幅0.45 m、長さ0.55 m）を
  別々に評価。footprint内のcell中心が長方形に入るcellを数える。
- 未観測・先読み条件に合わない地点は「安全」に数えず、coverageとして別集計する。
- 評価コード：`src/pm_evaluation/tools/evaluate_traversed_terrain.py`。

再実行する場合はFoxy containerで次を実行する（約4分、車両制御topicは再生しない）。

```bash
docker exec -i --user 1000:1000 --env HOME=/home/developer \
  patasmonkey_foxy_dev bash -s -- \
  /workspaces/patasmonkey_ws/bags/rosbag2_2026_07_26-09_17_38 \
  /workspaces/patasmonkey_ws/src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/traversed_path_1x_repeat \
  < src/pm_evaluation/tools/run_traversed_terrain_replay.sh
```

runnerはROS Domain 97を既定とし、必要なら`TERRAIN_EVAL_DOMAIN_ID`で変更できる。
元のbagとlocalization側のsourceは変更しない。

## 結果

| 指標 | 結果 |
|---|---:|
| 再計算odom sample | 10,186 |
| hazard map frame | 395 |
| 約0.25 m間隔の通過地点 | 427（軌跡長約112.1 m） |
| 2 m手前の地図が選べた地点 | 368 / 427 |
| footprintに1 cell以上観測があった地点 | 346 / 427 |
| footprintを全cell観測できた地点 | 196 / 427 |
| 中心が観測できた地点 | 325 / 427 |
| **中心が黒** | **37 / 325 = 11.4%** |
| footprintに黒cellが1個以上 | 207 / 346 = 59.8% |
| 全cell観測済みfootprintに黒cellが1個以上 | 99 / 196 = 50.5% |
| footprint内の黒cell率が10%以上 | 108 / 346 = 31.2% |
| footprint内の黒cell率が25%以上 | 64 / 346 = 18.5% |
| footprint内の黒cell率が50%以上 | 28 / 346 = 8.1% |
| 観測済みfootprint cell全体に占める黒cell | 3,869 / 32,239 = 12.0% |

footprintに黒が1つでもある地点のcomponent寄与（同一地点で重複あり）は、slope 191、
roughness 76、step 113、obstacle 65地点である。slopeが最も頻繁に関与した。

中心黒は27個の連続区間に分かれ、最長は軌跡の累積距離約2.4〜3.7 mの約1.3 mだった。
この初期区間が敷地と道路の境界に対応するかは、カラー画像との時刻照合が必要である。

## 読み方と限界

- 59.8%は「車体幅内に黒cellが**1つ以上**」という保守的な判定である。孤立cellも数える。
  11.4%は中心1 cellだけを見るため、実車幅を見落とす。両方を併記して判断する。
- 黒判定は4 cueの暫定閾値のいずれかが飽和したことを示し、衝突・走行不能のground truth
  ではない。ただし今回のような実走行済み経路での黒判定は、現行閾値をそのままplannerへ
  入れると通った道を塞ぎ得る、という明確な反例である。
- 約0.25 mごとの地点は空間的に相関するため、割合を独立試行の確率や信頼区間として
  扱わない。map上で観測が欠けた地点もあり、全427地点を同じ分母にしていない。
- footprint長0.55 mは評価用の暫定値で、接地輪の厳密な包絡形状ではない。幅0.45 mは
  車幅の概算。車体余裕幅・障害物へのmarginを加えると結果はさらに厳しくなり得る。
- 地図と実走行軌跡は同じ再計算odomを使用している。odomの共通系統誤差はこの比較で
  検出しにくい。RGB・現場観察との照合が別途必要。

## 保存データと再確認

集計値は`src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/
traversed_path_1x_final/summary.json`、各通過地点は同じ場所の`path_samples.csv`、
経路上の色分け図は`traversed_hazard.png`に保存した。これらは`results/`のため
gitignore対象である。このレポートは追跡可能な`notes/reports/`に置いた。

図では左に実走行軌跡の中心hazard（赤=黒判定、緑=非黒、灰=未観測/先読み不可）、
右に累積走行距離に沿ったfootprint内黒cell率と観測率を示す。図の点が密な区間でも、
CSVで時刻・odom座標・選択した過去地図時刻を確認できる。

## 次に検証すべきこと

1. 中心が連続して黒になった区間をカラー画像・RVizで確認し、実際の坂や路面段差と照合。
2. slope原因のcellについてground候補と局所平面の高さ分布を記録し、depth外れ値・姿勢誤差・
   異時刻融合のどれが主因か分解する。
3. robustなground/plane推定と空間的なsupport条件を導入し、同じ通過経路でfalse blackが
   どれだけ減るか再評価する。
