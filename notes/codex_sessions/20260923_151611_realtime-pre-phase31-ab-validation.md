User prompt: 念のため、変更前コードを１倍速条件で検証してください。

Codex final response: 検証しました。変更前のfeature実装を一時overlayへ復元し、リポジトリの現行コードは変更せず、別ROS Domainで7月bagを1倍速・最新localization再計算で全区間再生しました。

比較条件は揃っています。

| 条件 | 変更前 | Phase 3.1後 |
|---|---:|---:|
| 同期debug map数 | 394 | 395 |
| 評価済みhazardセル | 1,370,921 | 1,372,573 |
| black hazardセル | 440,669 | 438,513 |
| black率 | **32.144%** | **31.948%** |

差は **-0.196ポイント**（相対-0.61%）です。したがって、以前の結論どおり、Phase 3.1はhazardセル総数をほとんど減らしていません。

ただし内訳は明確に変わりました。black hazardセルへの寄与（重複あり）は以下です。

| cue | 変更前 | Phase 3.1後 |
|---|---:|---:|
| slope | 201,350（45.7%） | 282,458（64.4%） |
| roughness | 197,324（44.8%） | 140,442（32.0%） |
| step | 250,206（56.8%） | 139,483（31.8%） |
| obstacle | 179,456（40.7%） | 181,385（41.4%） |

変更前はstep由来の過剰反応が最大でした。平面残差とheading前後supportを導入した結果、step寄与は約25ポイント低下しています。しかし局所平面slopeが新しい最大要因になったため、hazard率全体はほぼ不変です。

このA/Bは、変更前だけ一時module overlayでcentral-difference slope・生の標準偏差roughness・生のmax-min stepを使い、他のdepth融合、TF、localization、YAML閾値、bag playerは同じままにしました。分析用ROSプロセスも停止済みです。

結論として、次の改善対象は明確にslope/ground候補です。最小z ground候補と異時刻融合の外れ値をrobustに抑えない限り、局所平面だけでは平坦舗装上の偽slopeを十分には抑えられません。
