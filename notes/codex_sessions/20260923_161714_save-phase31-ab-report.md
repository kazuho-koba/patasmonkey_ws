User prompt: さっきのA/Bテストの結果ですが、もう一度示すことってできますか？（テスト自体の再実行は不要）
コンソールに表示の上、レポートとして適切な場所に保存してほしいです。

Codex final response: 再実行せず、確定済みの7月bag・1倍速・最新localization再計算条件の結果を表示しました。

| 指標 | 変更前 | Phase 3.1後 |
|---|---:|---:|
| 同期debug map数 | 394 | 395 |
| 評価済みhazardセル | 1,370,921 | 1,372,573 |
| black hazardセル | 440,669 | 438,513 |
| black率 | **32.144%** | **31.948%** |

差は **-0.196ポイント**（相対 **-0.61%**）で、black hazard率は実質的に変わっていません。

一方、cue内訳は変わりました（black hazardセルに対する寄与、重複あり）。

| cue | 変更前 | Phase 3.1後 |
|---|---:|---:|
| slope | 45.69% | 64.41% |
| roughness | 44.78% | 32.03% |
| step | 56.78% | 31.81% |
| obstacle height | 40.72% | 41.36% |

平面残差とheading前後supportによりstepの過剰寄与は約25ポイント下がりましたが、slopeが新しい主因となったため、総black率はほぼ不変です。

再現条件、cell数、単独寄与、解釈、次の改善候補を含むレポートを [phase31_ab_1x_report.md](/home/kazuho/Projects/patasmonkey_ws/src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/phase31_ab_1x_report.md) に保存しました。
