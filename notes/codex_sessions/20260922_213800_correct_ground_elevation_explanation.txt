User prompt:
現在のURDFではcamera optical frameはbase_linkから約
  0.28 m上なので、base_linkの初期z=0かつ水平ならground elevationは概ね-0.28 mです。

といいますが、ということはground elevationはカメラ位置が基準の路面高さということではなく？

Codex final response:
はい、カメラ位置をゼロとする路面高さではありません。前回の`-0.28 m`という例は、地面がbase_linkと同じ高さにあるという不適切な仮定を含んでいました。訂正します。

`ground elevation`は、depthで得た路面点を撮像時刻の`camera optical frame → odom` TFで変換した、その点の**odom-z**です。カメラは観測原点ですが、出力zの基準にはしていません。

水平な単純化では、次の関係です。

```text
ground_elevation_odom
  = z_odom_base_link
  + z_base_link_camera
  - camera_to_ground_height
```

例えば、起動時に`z_odom_base_link = 0`、URDFのcamera高さが`+0.28 m`、
実際のレンズから路面までが約`0.40 m`なら、路面は`0 + 0.28 - 0.40 = -0.12 m`付近です。
この`-0.12 m`も「路面高」そのものではなく、base_linkを開始時にどのzへ置いたかで決まるodom座標です。

要点は次のとおりです。

- **camera-relative height**: cameraをゼロにしたときの路面の高さ。平面路なら概ね`-0.40 m`。
- **current ground elevation**: odomをゼロにしたときの路面z。camera-to-ground高さ、base_link高さ、localizer zのすべてを含む。

現在の実装は後者です。そのため、localizerのzが変わればground elevationも動きます。camera-relativeまたは「開始時の推定groundをゼロ」とする表示は、別layerとして明示的に追加するのが正しい設計です。
