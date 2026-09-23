User prompt: それを実行してもらえますか？Jetsonへの移植もあるので、計算量の増加には注意してください。これまで実装した分についても計算量削減の余地があれば、お願いします。
  -----------以下、やってほしいこと（引用）-----------------
  次に優先すべき改善は、局所平面をfitして残差だけをroughnessとすること、stepを平面残差・進行方向・観測support付きで定義し直すこと、各hazardセルの最大寄与cueを色分け表示することです。

Codex final response: Phase 3.1を実装しました。depth→rolling gridの融合hot path、timestamped TF、localizationには変更していません。

実装内容:

- 局所平面: freshな3×3近傍へ`z = ax + by + c`を最小二乗fitします。汎用の行列solverは使わず、固定stencilのnormal equationをNumPy配列上で累積し、閉形式（Cramer's rule）で解きます。
- slope: central differenceから局所平面の勾配`atan(sqrt(a²+b²))`へ置換しました。欠測・疎・共線な近傍はunknownです。
- roughness: 生の高さ標準偏差から、fit平面に対するRMS残差へ置換しました。滑らかな斜面はroughnessになりません。
- step: 生のmax-minから、fit平面に対する残差max-minへ置換しました。さらに`odom → base_link`の深度時刻姿勢から得るheadingについて、前後双方に既定2点以上のfresh観測が無いセルはunknownにします。
- 原因表示: 最大寄与cueを`/depth_elevation_mapper/terrain_hazard_cause_debug`へcodeとしてpublishします（1=slope、2=roughness、3=step、4=obstacle）。さらに、黒hazardセルを色付きcubeで重ねる`/depth_elevation_mapper/terrain_hazard_cause_markers`を追加しました。赤=slope、緑=roughness、黄=step、紫=obstacleです。

markerはJetsonで余分なmessage生成をしないため`publish_hazard_cause_markers: false`が既定です。bag確認時だけYAMLでtrueにし、RVizの`Hazard cause (colored, optional)`を有効化してください。最大2500 cubeへ均等間引きするので、debug有効時も無制限に増えません。

新しい主なparameterはYAMLにあります。

```yaml
step_min_side_neighbors: 2
publish_hazard_cause_markers: false
hazard_marker_max_points: 2500
hazard_marker_z_offset: 0.04
```

計算量への配慮:

- feature計算は従来どおり`debug_publish_rate`（既定2 Hz）だけで実行し、depth入力10〜12 Hzの毎フレームには実行しません。
- 120×120セルの固定NumPy arrayのみを使い、PointCloud2、SciPy、各セルのPython loop、バッチ`linalg.solve`を使いません。
- markerは既定OFF、必要時も黒hazardセルだけ・2500点上限です。
- 既存のpixel stride、入力rate上限、fixed-allocation ring grid、depthからgridへの直接融合も維持しています。

開発PCのFoxyコンテナで`pm_perception`と`pm_bringup`をビルドし、単体テストは`11 passed`です。120×120のfeature単体ベンチマークは約3.25 ms、7月bag高速replay中のmapperログではfeature平均約4.1 ms/更新でした。後者は2 Hzなので、通常時はmapper全体への追加CPU負荷を低く抑えます。Jetson実機上の最終値は、次回デプロイ後に同じmapperログの`feature ... ms mean`で確認してください。

`src/pm_perception/README.md`には表示方法と色の凡例を追記済みです。YAMLを変更した場合はconfigがinstall spaceへコピーされるため、再ビルド・source・mapper再起動が必要です。

```bash
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
cd /workspaces/patasmonkey_ws
colcon build --packages-select pm_perception
source install/setup.bash
```

今回のplane fitは最小二乗で、vegetationや深い外れ値にはまだrobustではありません。新しいfeatureが平坦舗装でどの程度改善するかは、通常速度のbag replayまたはJetson実測で確認してから閾値を再調整します。
