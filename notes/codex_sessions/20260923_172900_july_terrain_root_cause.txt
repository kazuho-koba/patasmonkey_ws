User prompt:
次は、中心が連続して黒になった区間をカラー画像と照合し、slopeが過大になった原因をground候補・depth外れ値・姿勢推定に分けて調べるのをお願いできますか？ただしslopeだけでなく、他の要因で黒だったケースについても同じように解析をお願いします。

平たんなアスファルト路面を走っていたしデプス画像もほぼそのとおりに映っていたようなのにこれほどhazard=1扱いになっていたのではperceptionとして使えないので、まずは原因究明をしたいです。

Codex final response:
調査しました。7月bagを1倍速で再生し、最新localizationを再計算したodomで走行経路を評価、連続して黒になった中心区間をカラー・depth画像と照合しました。

中心セルが観測済みの325地点中41地点（12.6%）が黒でした。slopeは31地点、roughnessは16、stepは20、obstacleは5地点に関与しており、重複があります。slope単独が16地点と最多ですが、step単独・obstacle単独も各3地点あり、実物のcone等が映るケースもあるため黒をすべて誤検出とは言えません。

舗装路に見える区間でも相対標高の局所3×3範囲が中央値9.6 cm、90 percentileで18.6 cmに達していました。現行mapperは5 cmセル内の最小zをground候補にする一方、外れ値をロバストに棄却せず、depthは4画素間引きです。この疎なground候補／depthノイズが局所平面のslope・roughness・stepへ伝播するのが、現時点で最も有力な原因候補です。ただし融合後のgridには元pixelや平面fit支持点を保存していないため、ground選択とdepth外れ値のどちらが各地点を作ったかはまだ確定できません。

姿勢については、黒地点のroll中央値1.61°（最大4.03°）、pitch中央値0.31°（最大1.69°）で、20°飽和を直接説明する大きな傾きは見えません。timestamp付きTFのdropもありません。一方、odom zは全軌跡で約-0.80〜+1.72 mと大きく変化し、VIO z jumpによるgate latchもログにありました。featureが使うrelative elevationでは各観測のcamera zを差し引くため、共通z offsetだけが今回の主因とは考えにくいものの、absolute elevation mapの別課題として残ります。

代表画像、cue組合せ別内訳、方法と限界、次の診断計画を[調査レポート](</home/kazuho/Projects/patasmonkey_ws/notes/reports/20260923_july_terrain_black_root_cause.md>)に保存しました。[カラー・depth画像の一覧](</home/kazuho/Projects/patasmonkey_ws/notes/reports/20260923_july_terrain_black_root_cause_images/contact_sheet.png>)と[時刻・画像対応JSON](</home/kazuho/Projects/patasmonkey_ws/notes/reports/20260923_july_terrain_black_root_cause_images/image_matches.json>)も確認できます。診断CSV等は`src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/forensic_1x/`にあります。

次に原因を確定するには、オフライン診断時だけraw depth pixel座標・depth値、セル内min/max/count、ground融合前後の値、局所平面のfresh supportと残差を保存する必要があります。今回の結果だけで閾値を緩めるのは、真の障害物も隠す恐れがあるので勧めません。なお、前回の報告にある中心黒37/325に対し今回41/325と少し差があります。再生回でodom/mapが完全一致しなかったため、この微差を改善・悪化とは解釈していません。
