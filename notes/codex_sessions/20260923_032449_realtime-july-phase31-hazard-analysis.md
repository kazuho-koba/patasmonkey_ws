User prompt: これでもう一度7月bagを評価して、hazard判定（黒）だった割合と、その原因となったのが４パラメータのうちどれであったかの割合の分析をしてください。
また分析に際して、今回新しく追加された４つのパラメータ（step_min_side_neighborsなど）の詳しい意味についても教えてください。

Codex final response: 現行のPhase 3.1実装で、7月bagを**1倍速・全229秒**再生して再評価しました。最新localizationを再計算し、同一timestamp・同一セルのhazardと4 component mapを照合しています。395枚の同期debug map、1,372,573個の評価済みhazardセルを集計しました。終端のTF extrapolationによるdropは1 frameのみです。

| 指標 | 結果 |
|---|---:|
| black hazardセル | 438,513 / 1,372,573 = **31.95%** |
| blackセルへ寄与: slope | 282,458 = **64.41%** |
| blackセルへ寄与: obstacle height | 181,385 = **41.36%** |
| blackセルへ寄与: roughness | 140,442 = **32.03%** |
| blackセルへ寄与: step height | 139,483 = **31.81%** |

寄与割合は重複を含みます。例えばslopeとroughnessが同じhazardセルを黒へできます。単独原因として最大なのはslopeのみ127,289セル（black hazardの29.03%）、次がobstacleのみ121,950セル（27.81%）です。roughness + stepだけは12,088セル（2.76%）で、旧方式に比べstepが過度に支配する状態は緩和されました。

結論として、平面残差・方向support付きstepへの置換後もblack hazard率は約32%で、現在の主因は**slope**です。平坦舗装路に対して20°以上のslopeがblackセルの64%へ関与するのは、走行可能性ではなく、最小z ground候補、depth外れ値、odom姿勢/高さの残留誤差、異なる撮像時刻の融合差に局所平面fitがまだ敏感なことを示します。`terrain_hazard_debug`を走行可否へ用いる段階ではありません。次に最優先なのは、ground候補のrobust化と、plane fit前の外れ値除去です。

今回追加した4 parameterは次です。

| parameter | default | 詳細 |
|---|---:|---|
| `step_min_side_neighbors` | 2 | 深度時刻の`odom → base_link` yawで定める車両headingについて、3×3近傍の前方・後方それぞれに最低何個のfreshセルが必要かです。不足時、stepはunknownでhazardへ寄与しません。残差max-min自体は全valid近傍から求めます。 |
| `publish_hazard_cause_markers` | false | 最大寄与cueを色付きRViz cubeとしてpublishするかです。falseならMarker作成・serializationが発生しません。 |
| `hazard_marker_max_points` | 2500 | markerを有効にした場合の黒hazard cube最大数です。超えた場合は均等間引きし、RViz/message負荷を上限化します。 |
| `hazard_marker_z_offset` | 0.04 m | 色付きcubeをelevation面から上へ浮かせる量です。地図面とのz-fightingを避ける可視化専用値で、hazard計算には影響しません。 |

mapperログではfeature計算は平均約4.2〜4.9 ms/更新、feature更新頻度は2 Hzです。markerは既定OFFのため、この測定にはmarker生成負荷を含みません。分析用ROSプロセスは停止済みです。
