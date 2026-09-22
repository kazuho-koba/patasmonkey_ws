User prompt
その前に質問で、VIOについてはx,y,zではなくvx, vy, vzのみを使うということもありえますか？
その方が破綻後もVIOノードのリセット→GNSS情報などを基点に相対運動量だけでオドメトリ推定を継続、、、というのがやりやすいのではないかという仮設です。既に加味してあるのであれば放念ください。

Codex final response
はい。VIOの `vx, vy, vz` だけを融合する方式は可能で、今回の問題に対して有力です。

実は従来の `ekf_local_whl_imu_cam.yaml` は、すでにVIO poseを使わず、`/vio/odometry` の `vx, vy, vz` を融合しています。また過去の7月A/Bでも、`wheel vx + Wit wz + VIO vx/vy` を試しています。pose resetを位置へ直接入れないため、今回確認した「VIO pose zの不連続がEKF水平状態まで壊す」問題は避けられます。

ただし7月bagでは、VIO速度を加えた条件の始終点誤差は6.28 mで、wheel vx + Wit wzだけの0.85 mより悪化しました。つまり「poseより安全」ではありますが、VIO velocityの遅延・横速度・共分散・wheel vxとの二重観測は調整が必要です。

御仮説の「VIOがリセットしても、現在のEKF位置を維持し、相対速度だけで継続し、global EKFがGNSSで補正する」は成立します。VIO pose resetは無視でき、local EKFは積分を継続し、GNSSはglobal EKF側で絶対位置を補正します。

ただし次の2点は必要です。

- VIOの内部リセット後、速度が正常化したことを確認してから再受理する。8月ではVIO速度自体も異常化したため、pose用のラッチゲートとは別に、速度ノルム・z速度・加速度変化・時刻連続性を検査するtwist用ゲートが必要です。
- `vz`だけでは絶対的な高さを観測できません。高さは積分値となるため、長時間のz精度にはVIO pose z、GNSS高度、地形情報など別の位置観測が必要です。

したがって次の実装候補は、まず安全な水平系として `wheel vx + Wit roll/pitch + Wit wz + gated VIO vx/vy` を検証し、`vz`は独立した高さ推定系で扱うことです。VIO poseを使わないtwist専用ゲートは、pose resetを契機に永久遮断せず、短い健全性確認区間の後に再受理できるようにします。
