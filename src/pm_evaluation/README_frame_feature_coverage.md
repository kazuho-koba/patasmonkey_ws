# フレーム内指標の保持によるA相当coverageのoffline評価

通常perceptionのnode・parameter・launchは変更しない。別Pythonスクリプトで、
同じ入力列・実際のmapper TF・intrinsicsを使い、独立画像／指標保持／高さ時間融合を
一度のbag走査で比較する。センサ・モータ・ROS launchを起動しない。

## 実行

Foxy開発コンテナでoverlayをsourceする。toolsスクリプトの追加だけなのでbuild不要。
必要なNumPy・ROS message・MCAP等は既存独立フレーム評価と同じ依存を使う。

```bash
cd /workspaces/patasmonkey_ws
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
source install/setup.bash
python3 src/pm_evaluation/tools/evaluate_frame_feature_coverage.py \
  bags/rosbag2_2026_07_26-09_17_38 \
  --mapper-trace notes/reports/perception/20261003_100000_depth融合_stride比較_assets/stride1/trace/mapper_trace.jsonl \
  --output /tmp/new_frame_feature_coverage \
  --stride 1 --single-frame-obstacle \
  --max-age 3 --lead-seconds 0.2 1 2
```

BLASの多thread競合を避ける場合はPython起動前に`export OPENBLAS_NUM_THREADS=1
OMP_NUM_THREADS=1`を指定する。今回の確定測定はこの条件。計算方式・入力は変えない。

出力先は存在しない新規ディレクトリを指定する。traceのmetadataから全条件を取得し、
別YAMLを混ぜない。条件変更は対応する設定でTF採取し直したtraceを使う。
`--stride`だけは明示的な比較因子で、既定1。stride=1は最近追加した代替画素探索の
影響を受けない。その他のbagには、先にそのbag用のmapper traceを採る必要がある。

`--single-frame-obstacle`は独立／保持側だけでconfidence待ちを省略する。
高さ融合側は通常のconfidence gateを維持するので、obstacle規則の差を含む。
このフラグを外すと3方式で通常confidence gateを使えるが、独立画像では通常の
0.1証拠が0.15既定閾値に届かず、obstacleが未評価になりやすい。
低黒率の差を高さ融合の効果だけに帰属しない。

## 処理と主要parameter

1. traceに記録された画像だけを採用し、保存されたcamera→odom TFで投影する。
2. 各画像を空gridへ入力し、既存feature計算で4指標を計算する。
3. 指標が有効なセルだけ更新する。unknownは過去の有効値を消さない。
4. cueごとに保持時刻を記録し、締切時点で`--max-age`秒を超えた値をunknownにする。
5. 同じtraceのbase poseで経路を作り、各位置の通過より`--lead-seconds`秒前に評価する。

`held_coherent_terrain`対照も同時出力する。slope/roughness/stepを同じframeで全て
評価できた組だけ更新・保持するため、3cueの取得frameが異なる組み合わせを避ける。
obstacleはこの対照でも独立に更新する。これは同一画像でfootprint全体や全継ぎ目を
観測できた保証ではなく、各セルの3cue取得frameを揃える条件である。

高さ融合・空間補間・z補正・E/Fは指標保持側へ入れない。
`--path-spacing`既定0.25 m、footprintは`--width`0.45 m×`--length`0.55 m。
odom上のcell中心がこの長方形に含まれるセルを分母にする。現在map外もunknownとして
数える。経路はtraceの撮像時刻base poseであり、高頻度odom CSVと完全一致はしない。

独立側は締切以前の最新1画像、高さ融合側は同じ最新画像まで融合したgridを参照する。
3方式とも同じ締切・同じfootprintで比較し、締切と同stampは利用可、未来stampは利用不可。
経路poseを未来から使うのは評価対象を定めるためだけで、特徴推定には使わない。

## 出力と分母

- `summary.json`：条件、採用数、締切別の3方式比較、CPU時間・elapsed・peak RSS・処理時間。
- `footprints.csv`：同一通過位置・締切の各指標coverage、黒、unknown、保持age、接続proxy。

hazard既知は少なくとも1cueが有効、黒は有効cueのどれかがround後100になること。
黒率の分母は1セル以上hazard既知のfootprint。全unknownは分母から除くが件数を併記する。
地形3指標(slope/roughness/step)有効率と4指標有効率を別に出す。
セルvalid率は延べセルで、footprint完全評価の割合とは別である。
障害物高さ幅が記録閾値未満ならobstacleはNaNなので「障害物なし確認済み」とはしない。

保持側の`coherent_terrain_cells`は3cueが同じframeで最後に更新されたセル数。
`no_fresh_joint_support_pairs`はfootprint内の4近傍で両端地形3cueが既知でも、
保持期限内の同一画像で両端3cueを評価した履歴がないpair数。
`different_source_pairs`は両端の最新cue時刻が異なるpair数で、危険数ではない。
joint supportは接続の観測support proxyであり、本物の継ぎ目段差の計算ではない。

初回3cue取得のlead中央値/p05は、各cue初回stampの最大と通過stampの差。
初回取得後の期限切れやframe間coherenceは保証しないので、締切別coverageと最新ageを
合わせて読む。hazard＝安全、coverage＝十分な停止距離という保証ではない。

## 制約

これはlatest-valid上書きによるcoverage prototypeで、危険証拠を安全に融合する完成版ではない。
指標の向き(stepの評価方向)は観測時のyawであり、別方向からの再走行にそのまま適用できない。
期限内でもpose誤差、動的物体、誤対応は残る。継ぎ目のz補正や幾何検証は行わない。
通常runtimeのhazard 2 Hzを再現せず、各採用画像で独立評価して保持する。
時間基準は撮像stamp。実機callback待ち・fusion/評価完了遅延を引いていないため、
実際にその情報が利用可能になった時刻や停止余裕を保証する評価ではない。
CPU値はofflineの3方式同時計算で、Jetson runtimeの負荷に換算しない。
