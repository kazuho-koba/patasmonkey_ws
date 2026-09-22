User prompt:
notes/screenshotsに入っている最新の画像を確認してください。
.yamlはmap_sizeを5*5に変更、max_depthを4mから6mに変更しています。
グレーの四角形は概ね5m四方に見えますが、実際の/depth_elevation_mapper/elevation_debug表示が明らかに幅が狭く、目測3mくらいしかないようにみえるのですが原因はなんですか？

Codex final response:
原因は設定が反映されていないことではありません。画像の外側の灰色の正方形が5 m × 5 mのrolling windowで、内側の色が付いた台形状の部分だけが、カメラdepthで実際に観測されているセルです。

現在のwindowは`base_link`中心です。したがって`map_size_x: 5.0`では前方2.5 m・後方2.5 mまでしか保持できません。`max_depth: 6.0`は「6 mまでのdepth値を捨てない」条件であり、mapを前方6 mまで広げる設定ではありません。5 m windowの前端では、OAK-Dの水平画角（およそ58度）から、観測できる横幅は概算で次の程度です。

```text
2 × 2.5 m × tan(58° / 2) ≒ 2.8 m
```

そのため、目測約3 mの有効elevation幅は想定どおりです。残りの灰色セルはunknown（観測なし）であり、groundを5 m四方に埋めたり、costを表しているわけではありません。

また、遠方のgroundは画面下側の狭い帯にしか写らず、stereo depthの有効率も落ちるため、`max_depth`を6 mにしても全面が埋まるわけではありません。

前方6 mを地図として保持したい場合、現状のcentered windowでは`map_size_x: 12.0`が必要です（前方6 m＋後方6 m）。実際には後方をそこまで必要としないため、次の実装拡張では「後方1 m・前方6 m」のようなforward-biased rolling window用の`map_center_offset_x` parameterを追加するのが、メモリと観測範囲の両面で適切です。
