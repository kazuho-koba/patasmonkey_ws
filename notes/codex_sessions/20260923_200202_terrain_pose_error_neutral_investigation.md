User prompt:
オドメトリ推定の誤差が大きいという可能性もあるのでしょうか？引き続き、原因を予断せず究明を続けてもらえますか？

Codex final response:
はい、オドメトリ誤差もまだ候補です。今回の27.7 cm差については、少なくともbase zの平行移動だけでは説明できませんが、姿勢（特にpitch/roll）、水平位置・yaw、時刻整合の誤差までは除外できていません。

7月bagの旧・新depth画素をRGB/depth画像へ戻して確認しました。代表セル `(152,124)` の旧候補 `(160,276)` と新候補 `(224,356)` は、RGB上ではどちらも舗装路に見え、目視できる大きな段差はありません。ただし「同じodomセル」は推定投影上の一致であって、実世界で同一地点を見た独立な証拠ではありません。旧点はdepth有効領域の上端近くです。一方、旧候補のframe内sample 4点の高さ幅は約2.4 cm、新候補3点は約2.8 cmで、単一の孤立ノイズというより、各frameの候補群全体が異なる高さへ投影されていました。

旧・新source間のbase z差は約1.3 cmですが、pitchは約2.03°、yawは約14.45°変わっています。現行replayの構成ではroll/pitchはWIT IMU、yawは水平EKF由来です。RGB画像でも視点変化は見えますが、画像だけで姿勢が正しいとは判定できません。また7月bagにCameraInfoはなく、mapperは設定済みEEPROM fallback intrinsicsを使用しています。depth候補選択、姿勢・XY推定、intrinsics/外部パラメータ、時刻差のどれかに決めつけるだけの証拠はまだありません。

比較画像は[ground source contact sheet](</home/kazuho/Projects/patasmonkey_ws/src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/revisit_ground_inputs_1x_clean/ground_source_images/ground_source_contact_sheet.png>)、解析上の注意と再現コマンドは[レポート](</home/kazuho/Projects/patasmonkey_ws/notes/reports/20260923_july_terrain_odom_z_revisit_analysis.md>)を参照してください。画像抽出用に [inspect_ground_source_images.py](/home/kazuho/Projects/patasmonkey_ws/src/pm_evaluation/tools/inspect_ground_source_images.py) を追加し、bagから7件の置換例を抽出しました。スクリプトの構文確認と `git diff --check` は成功しました。診断結果は引き続き選択ROIのもので、bag全体の原因頻度ではありません。

次は、候補周辺のRGB-D特徴から相対カメラ姿勢を独立に推定し、replayのodom変位・姿勢変化と比較するのがよさそうです。これにより、少なくとも「画像に現れたカメラ移動」と「localizationが報告した移動」が大きく食い違っているかを調べられます。
