User prompt:
これらの値はパラメータとして変更可能になっていますか？

Codex final response:
はい。以下はいずれも`src/pm_perception/config/depth_elevation_mapper.yaml`のROS parameterです。

| parameter | 現在値 | 用途 |
|---|---:|---|
| `map_size_x` | `6.0` m | rolling mapのX方向（前後）サイズ |
| `map_size_y` | `6.0` m | rolling mapのY方向（左右）サイズ |
| `resolution` | `0.05` m | セルサイズ。6 mなら120×120セル |
| `min_depth` | `0.4` m | 使用する最短depth |
| `max_depth` | `4.0` m | 使用する最長depth |
| `pixel_stride` | `4` | depth pixelの間引き幅 |

たとえば前方をもう少し保持したい場合も、現在の実装はbase_link中心の正方windowなので、まず`map_size_x: 8.0`、`map_size_y: 6.0`のように変更できます。ただし深度入力は`max_depth: 4.0`のままでは4 mより遠方を新規観測できません。遠方も観測したい場合は`max_depth`を増やしますが、OAK-D depthの遠距離ノイズと処理対象点が増えるため、bagで品質を確認してから段階的に上げるのが安全です。

変更後は、`pm_perception`を再ビルドする必要はありません。launch時にYAMLを読むので、起動し直せば反映されます。
