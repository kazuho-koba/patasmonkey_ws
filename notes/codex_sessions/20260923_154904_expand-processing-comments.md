User prompt: コメントについては非自明な実装意図などのハイレイヤな話だけでなく、実際に実行している処理に関する説明についても追加してもらえますか？
またその旨、AGENTS.mdも追記・修正してほしいです。

Codex final response: 対応しました。[AGENTS.md](../../AGENTS.md) のコメント方針を、実装意図だけでなく「入力・単位、filtering、座標変換、中間値、branch/timeout、状態更新」という実処理の説明も必須とする内容へ更新しました。

既存実装にも、実際のデータフローが追えるコメントを追加しています。

- [depth_projection.py](../../src/pm_perception/pm_perception/depth_projection.py): ROS Imageのrow padding処理、mm→m変換、depth範囲filter、pinhole back-projection、row-vectorへのTF適用順序。
- [rolling_elevation_grid.py](../../src/pm_perception/pm_perception/rolling_elevation_grid.py): odom cellからring-buffer slotへの変換、frame内min/max集約、slot再利用時のclear、ground更新の分岐、重み付き平均・分散更新、obstacle heightの実際の算出、publish用logical map copy。
- [terrain_features.py](../../src/pm_perception/pm_perception/terrain_features.py): fresh mask、各offset stencilからnormal equationを作る処理、残差の再走査、heading前後support判定、missing cueをunknownのままhazard合成する処理。

既存のhigh-levelコメントと重複するだけの記述は追加していません。`git diff --check`および対象Pythonファイルの`py_compile`は通過しています。コメントのみの変更なのでROS buildは不要です。
