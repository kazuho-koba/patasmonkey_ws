User prompt: その方針にしたがってこれまで実装した分についてコメントを適宜追加してください。既に実施済みであれば不要です。

Codex final response: コメントを見直し、不足していた箇所だけへ追記しました。動作・topic・parameterは変更していません。

主な追加先は以下です。

- `src/pm_perception/pm_perception/depth_elevation_mapper_node.py`
  - depth撮像時刻のexact TFを守るためのbounded FIFO、古いframeを捨てる理由
  - `odom` elevationとcamera対地高で正規化したrelative elevationの役割の違い
  - 同一撮像時刻のbase poseでrolling mapをrecenterし、yawはstepの観測support判定だけに使うこと
  - debug publish、OccupancyGrid、Marker、PointCloud2を低頻度／既定OFFにしてJetsonのhot pathから外している理由
- `src/pm_perception/pm_perception/depth_projection.py`
  - ROS optical frameの軸・単位、16UC1 mm depth、strideによる2軸間引き、payload viewと作業用配列の扱い
- `src/pm_perception/pm_perception/terrain_features.py`
  - headingがstep量の符号付けではなく前後supportのgateにだけ使われること
  - hazard cause同率時の優先順と、それが「唯一の原因」を意味しないこと
- `src/pm_evaluation/pm_evaluation/cli/bag_clock_player.py`
  - typed topic whitelist、`/tf_static`のtransient-local QoS、開始前のdiscovery待ちの意図

`rolling_elevation_grid.py`はground/obstacle仮説、重み減衰、ring-buffer再利用を既に十分説明しており、不要な重複は追加していません。`pm_bag_global_localization.launch.py`も記録topicの目的が既にコメント化されています。

`git diff --check`と対象Pythonファイルの`py_compile`は通過しました。コメントのみの変更なのでROS buildは実施不要です。
