User prompt: 現時点でのslope, roughness,step, obstacleの詳しい計算方法がわかっていないのですが、readmeやスクリプトのコメントに書いてありますか？

Codex final response: はい、実装側には一部コメントがありましたが、READMEの説明は概要に留まり、4指標の式・入力・有効条件が一箇所に揃っていませんでした。今回、現在の実装と一致する詳細を README に追記しました。

[`README.md`](../../src/pm_perception/README.md) の「Stage 3.1 の4指標の計算方法」に、以下を明記しています。

- slope: 3x3 freshセルへの最小二乗平面 `z=ax+by+c` の勾配角。上り/下りの符号は持ちません。
- roughness: 同じ平面からの残差RMS。生の高さばらつきではありません。
- step: 平面残差のmax-min。heading前後それぞれに `step_min_side_neighbors` 以上の観測がある場合だけ有効です。headingは現時点ではsupportの確認だけに使います。
- obstacle: 同一セルのdepth最小zを暫定ground、最大zとの差を垂直extentとして蓄積する軽量cueで、confidenceの時間減衰も含みます。意味的な物体分類ではありません。

また、hazardの正規化式、unknownになる条件、複数cueが同率の場合のcause表示優先順、現在の制約も追記しました。古い「roughness=高さ標準偏差、step=max-min」というREADME表記も、現行の局所平面残差方式へ修正済みです。

実装コメントは [`terrain_features.py`](../../src/pm_perception/pm_perception/terrain_features.py) と [`rolling_elevation_grid.py`](../../src/pm_perception/pm_perception/rolling_elevation_grid.py) にあります。今回はドキュメント変更のみなので、再buildは不要です。
