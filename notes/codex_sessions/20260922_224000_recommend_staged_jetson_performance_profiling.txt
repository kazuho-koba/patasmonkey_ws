User prompt:
この処理がjetsonでも重すぎないかを調査したいのですが、phase3まで実装してからの方がいいでしょうか？
今までの実装は全てjetsonにもデプロイ・ビルド済みです。

Codex final response:
Phase 3を待たず、まず現在のStage 2をJetson上で基準計測するのが適切です。現状のdepth投影・TF・grid融合が基準になり、Phase 3のslope/roughness追加後に増分を正確に比較できます。

推奨順序は次です。

1. Stage 2の標準設定（まず6 m × 6 m、`max_depth: 4.0`、`pixel_stride: 4`）をbagで再生し、処理時間、frame drop、CPU、RSSを記録する。
2. 想定設定（例: 12 m windowや`max_depth: 6.0`）も同じbag区間で測り、mapサイズ・有効点数がどの程度効くか確認する。
3. Phase 3を、ground平面・slope/roughness/stepを一度に入れず、機能ごとに追加して同じ条件で再計測する。

現実装は、depthの全画素PointCloud2を生成せず、4画素間引き・NumPy一括投影・固定長ring bufferを使っています。x86開発環境での短い7月bag smokeでは有効約5,100〜5,500点/frame、約530〜580セル/frame、平均約2.8 ms/frameでしたが、これはJetsonの性能保証ではありません。Jetsonでの実測を基準にします。

mapperの5秒周期ログには平均/最大処理時間、sample数、cell数、rate drop、TF dropが出ます。並行してJetsonで`tegrastats --interval 1000`を実行し、CPU使用率・RAM・温度・GPU使用率を記録してください。`ros2 topic hz /oak/depth/image_raw`と`ros2 topic hz /depth_elevation_mapper/elevation_debug`も確認対象です。

評価中は、`publish_debug_pointcloud: false`を保ち、Stage 2 debug mapが不要なら`publish_stage2_debug_layers: false`、全debugを止めるなら`publish_debug_occupancy: false`にします。これでcore fusionと可視化message生成の負荷を分けて比較できます。

なお、source側のYAMLを手で変更した場合、このworkspaceの通常installではconfigがinstall spaceへコピーされるため、Jetsonでも`colcon build --packages-select pm_perception`とmapperの再起動が必要です。先ほどのhost側の再ビルドは中断されたため、install側へ12.0設定が反映済みとは扱わないでください。
