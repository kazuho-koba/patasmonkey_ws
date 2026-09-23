User prompt: preliminary terrain hazardというのはどのような情報ですか？

Codex final response: `Preliminary terrain hazard`は、最終的な走行コストではなく、Stage 3で追加した「局所地形が荒れていそうか」を確認するための暫定デバッグ指標です。

各セルについて、次の4つをそれぞれ閾値で0〜1へ正規化し、最大値を取っています。

- slope: 20°で1
- roughness: 3 cmで1
- step height: 10 cmで1
- obstacle height: 20 cmで1

RVizでは0が黒、1が白です。つまり白いセルは「4指標の少なくとも一つが現在の暫定閾値以上」を意味します。例えば、平坦でもdepthのばらつきが大きければroughness由来で白くなり得ますし、緩やかな斜面でも3×3近傍の高低差が10 cmを超えればstep由来で明るくなり得ます。

これは以下を意味しません。

- Nav2へ渡すcost
- 通行不可という判定
- 人・岩・草・negative obstacleの分類
- 信頼できる完成版traversability

また、観測が古い（既定3秒超）・近傍のfreshな観測が少ないセルは、危険=白にはせずunknownにしています。まず各component topic（`slope_debug`、`roughness_debug`、`step_height_debug`、`obstacle_height_debug`）を併せて表示し、何がhazardを押し上げたかを確認するための表示です。
