User prompt:
次に原因を確定するには、オフライン診断時だけraw depth pixel座標・depth値、セル内min/max/count、ground融合前後の値、局所平面のfresh supportと残差を保存する必要があります。

では、これえを実行してください。

Codex final response:
実行しました。offline replay時だけ有効になるprovenance記録を追加し、7月bagで取得・集計しました。通常のJetson runtimeでは無効のままで、追加のprovenance配列、CSV書き込み、ROS message生成は行いません。

記録するようにした情報は、間引き後のraw depth pixel座標・axial depth、セル内sample数とworld-z min/max、その代表pixel、ground/relative-groundの融合前後値とfusion mode、depth撮像時のbase姿勢、局所平面係数・残差、平面fitに使ったfresh supportごとの残差と元pixel/depthです。全画素や全履歴を貯めず、危険セルとその近傍だけを対象にします。

7月bagの完了済みROI replayでは、中心黒地点37点中30点を同一時刻・同一odom cellで対応付けました。これらではslopeが20点、roughnessが17点、stepが21点、obstacleが4点で閾値を超えていました（重複あり）。slope+roughness+stepが同時に超過した地点が12点、slope単独が7点などで、ひとつのcueだけが原因ではありません。

重点provenance再生ではさらに21地点のraw pixel/support記録を得ました。ただし先行評価との完全一致は21/37点で、重点再生の出力にrunnerの最終summaryがないため、37点すべてをカバーした再生としては扱っていません。全体のcue統計は完了済みROI replay、pixel由来の詳細はこの21地点を対象にした結果です。

重要な具体例では、近傍セルの今回のdepth候補はworld-z約-0.161 m（pixel `(392,340)`、depth約2.011 m）でしたが、融合器には過去のground値約-0.405 mが残っていました。差がmerge閾値0.20 mを超えたため今回の候補は棄却（fusion mode 0）され、隣接セルとの間に約22 cmの相対標高差が残っていました。これは局所slope/stepを大きくしうる、ground履歴由来の不連続が実データにあることを示します。

同一frame・同一cell内のworld-z幅は中心黒地点で中央値約1.6 cm（90 percentile約3.6 cm）でした。一方、fresh supportと局所平面の絶対残差は中央値約2.4 cm、90 percentile約8.3 cm。瞬時roll/pitchも過大な平面gradientを説明する大きさではありません。したがって、pixel単体の散らばりだけよりも、誤った低ground候補の保持や複数時刻の地図融合が強い疑いです。ただし、最初の低ground値がdepth外れ値、TF/姿勢誤差、地物のどれから来たかは未確定です。raw pixelとbag画像を対応付けて確認する必要があります。

調査結果と制約は[診断レポート](/home/kazuho/Projects/patasmonkey_ws/notes/reports/20260923_july_terrain_forensic_provenance.md)にまとめました。主なデータは[全体集計JSON](/home/kazuho/Projects/patasmonkey_ws/src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/provenance_1x_retry1/forensic_summary.json)、[重点集計JSON](/home/kazuho/Projects/patasmonkey_ws/src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/provenance_targeted_1x/forensic_summary.json)、各runの`forensic/hazard_cells.csv`と`forensic/plane_support.csv`です。

変更は`pm_perception` mapper/grid/projection/feature診断、replay launch、READMEと、集計ツールです。Foxy buildと14 testsは実装中に成功し、今回の集計ツールはPython compile、runnerは`bash -n`、差分は`git diff --check`で確認しました。次はレポートのtimestamp・pixel座標からbagのcolor/depthを照合し、最初の低いground候補を作った要因を特定する段階です。
