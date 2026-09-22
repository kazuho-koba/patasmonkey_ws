User prompt:
ここでいうground elevationは計測開始時をゼロとした相対的な路面のz座標くらいの意味でしょうか？

Codex final response:
ほぼ「相対座標」ですが、正確には「開始時の路面をゼロ化した値」ではなく、`odom`原点に対する路面のzです。初期路面を明示的にゼロにする処理はまだ入れていません。

現在の計算は次のとおりです。

```text
depth point (camera optical frame)
  └─ depth撮像時刻の TF camera → odom で変換
       └─ その変換後の z = ground elevation
```

したがって基準は`odom`のz原点です。localizationが起動時に`base_link` zを0付近に置けば、静止している平面路面は概ね`-base_link→cameraの高さ`付近になります。現在のURDFではcamera optical frameはbase_linkから約0.28 m上なので、base_linkの初期z=0かつ水平ならground elevationは概ね`-0.28 m`です。実際にはroll/pitch、RGB optical frameの変換、localizer zにより変わります。

つまり「初期時刻の地面との差」を見たい場合は、現在の表示をそのまま解釈せず、次のどちらかをStage 2で追加するのがよいです。

- map内の最初に安定したground平面/中央値を`z=0`として差分elevationを出す
- `base_link`またはローカルground planeを基準にしたrobot-relative elevation layerを別に出す

前者はodom上で過去地点を比較する地図、後者はその瞬間の踏破性評価に向きます。今回の`elevation_debug`は前者・後者のどちらにも正規化していない、odometry依存の生のodom-z表示です。
