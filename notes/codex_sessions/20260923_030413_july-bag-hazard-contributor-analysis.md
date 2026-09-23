User prompt: いつも使っている7月bagファイル（/workspaces/patasmonkey_ws/bags/rosbag2_2026_07_26-09_17_38/）で、hazardが黒になった理由として最も頻度が高いのは、４つのパラメータのどの閾値を超えたことによるのが、分析できますか？正直、どのパラメータで可視化してもそこまで真っ黒というわけではないし、実際にもほぼ平坦地（家の敷地から道路に出るときに少し下り坂がある程度）なので、hazardだけかなり黒く見えるのに違和感があります。

Codex final response: 分析しました。7月bag全区間を、現在の「最新localization再計算 + Phase 3 mapper」で再生し、hazard mapと4 component mapの同一timestamp・同一セルを照合しました。実行環境の制約から10倍速再生ですが、395枚の同期debug mapを取得できています（本来の約458枚の約86%）。傾向の判断には十分な標本です。

結果は次のとおりです。

| 項目 | 結果 |
|---|---:|
| 評価済みhazardセル | 1,371,487 |
| black hazardセル | 440,725（32.1%） |
| blackセルで最も多く閾値到達したcue | **step height: 251,358（57.0%）** |
| 次点 | slope: 199,688（45.3%） |
| 次 | roughness: 197,493（44.8%） |
| 次 | obstacle height: 180,013（40.8%） |

この4行は重複を含みます。例えばstepとroughnessが同じ黒hazardセルへ同時に寄与できます。単独で黒にした最大の組合せは`obstacle height`のみで116,234セル（black hazard全体の26.4%）でした。次に多い組合せは`roughness + step`の78,814セル、`slope + roughness + step`の69,949セル、`slope`のみの63,227セルでした。

したがって、「最も頻度が高い閾値」はstep height（7 cm）ですが、black表示の大半は単一原因ではありません。step/roughness/slopeが同時に閾値到達しているセルが多く、obstacleだけの黒も相当数あります。

今回の現在設定ではcomponent表示上限を閾値と一致させているため、閾値以上のcomponentはRVizでも黒です。それでもhazardだけ黒く見えた以前の画像は、componentの表示上限が閾値より高かった旧設定で見たものです。現在のYAMLと再ビルド済み環境で再生すれば、hazard=黒の箇所には少なくとも一つ黒いcomponentが対応します。

ただし、平坦な舗装路でstep / roughness / slopeがこれほど頻繁に閾値へ達するのは、現Stage 3のfeature定義がまだ走行可否判定に粗すぎることを示しています。

- `step height`は3×3（約15 cm角）の単純なmax-minなので、depth外れ値・疎なセル・姿勢/高さの残留誤差・実際の小さな路面粗さを段差として拾います。
- `roughness`は局所平面を引かない標準偏差で、路面の緩い傾斜や同じ観測誤差にも反応します。
- `slope`も`relative_elevation`の局所差分なので、odom姿勢誤差やrolling mapへ融合した異なる視点の観測差を含みます。
- obstacleは同一5 cmセルのmin/maxに基づく最小実装で、塀に反応する一方、ground更新の差を拾う余地があります。

従って、現時点の`terrain_hazard_debug`を「平坦舗装路の32%が走行不能」とは解釈しません。これはStage 3の初期cueがセンサ・融合誤差へどれだけ敏感かを表す診断結果です。次に優先すべき改善は、局所平面をfitして残差だけをroughnessとすること、stepを平面残差・進行方向・観測support付きで定義し直すこと、各hazardセルの最大寄与cueを色分け表示することです。
