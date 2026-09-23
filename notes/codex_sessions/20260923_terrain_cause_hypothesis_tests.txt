User prompt:
ありがとうございます、では、仮説を１つずつ検証してもらえますか？
現状どのような課題があって、それに対して設定した仮説と、なぜそのような仮説が置けたかの根拠、そして検証方法と検証結果は、いつものプロンプト-出力のまとめとは別にレポートとしてまとめてください。

Codex final response:
仮説ごとの根拠・検証方法・結果・未確定点を、独立した[原因仮説検証レポート](</home/kazuho/Projects/patasmonkey_ws/notes/reports/20260923_july_hazard_cause_hypothesis_tests.md>)にまとめました。従来の解析レポートからもリンクしています。

現時点で確認できたのは、代表セルでground候補が約29 cm低く置き換わり、その差分がobstacle cue/hazard=1になった経路です。一方、単発depth spikeやframe内の最小値選択だけでは差を説明しにくく、base z並進差も約1.3 cmでした。画像上の旧・新pixelは舗装路に見えますが、同じodom cellは実世界の同一点を意味しません。

撮像時刻近傍のWIT IMUとmapper poseは数ms以内で一致しており、stale IMU姿勢という時間ずれ仮説は支持されません。ただしIMU姿勢の真値性は未検証です。RGB-D相対姿勢PnPは対応数・再投影誤差の品質条件を満たさず棄却したため、その出力からodom誤差を判定していません。また7月bagにはCameraInfoがなく、fallback intrinsicsの現物校正も未確認です。

よって、odom誤差、投影上で別surfaceを同一cellへ重ねた可能性、depth/camera calibrationは未確定のままです。平均化filterのA/B試験もまだ行っていません。次はOAK-Dのdepth/RGB alignmentとfallback校正値の照合、品質のよい区間に限定した独立相対pose検証を行い、その後に集約方式や時間filterを比較する方針です。

画像抽出・source画素検査用の `inspect_ground_source_images.py` と相対姿勢比較用の `compare_ground_source_visual_rotation.py` を追加しました。出力は `src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/revisit_ground_inputs_1x_clean/` 以下です。両スクリプトの構文確認、JSON読込確認、`git diff --check` は成功しました。
