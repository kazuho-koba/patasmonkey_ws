User prompt
~/Projects/patasmonkey_ws/src/pm_evaluation/results/separated_offroad_localization/rosbag2_2026_07_26-09_17_38_stable4s_rate10
および ~/Projects/patasmonkey_ws/notes/codex_sessions/20260922_020109_implement_separated_offroad_localizer.txt
を見ると、

- 7月bag: 最終水平位置は `(6.39, -1.93) m`、高さは`-0.799 m`。VIOのzが1.727 m段差でリセットした時点で高さゲートが閉鎖しました。速度ゲートは4秒の安定化後に再投入し、後段の速度不連続（1.902 m/s）で隔離してから、再び4秒の安定化を要求しました。

ということを言っていますが、実際にグラフseparated_localization_safe_output.pngを見ると、zの推定自体は継続していますよね。ゲート閉鎖によって何が変わったんですか？

Codex final response
ご指摘のとおり、図の右側は「z出力」が最後まで続いています。ただし、ゲート閉鎖後にzを新たに推定・追従しているのではなく、最後に通過した値を保持してIMU周期で再publishしています。

根拠はCSVと実装です。

- 生VIOでは、`t_rel=137.990 s`でzが`-0.798975 m -> +0.927716 m`へ、`1.726691 m`不連続に変化しています。
- `vio_vertical_gate_node`はこの値を転送せず、その時点でラッチ閉鎖しました。以後は`/vio/odometry/gated`に新しいzをpublishしません。
- `attitude_height_observer_node`はIMU callbackごとに`/odometry/local_vertical`を出し続けますが、height callbackが来なくなると内部の`self.height`を更新しません。そのため最後の妥当値`-0.798975 m`を保持します。
- 実際にobserver CSVでは、zの最終更新は` t_rel=155.179 s`、以後最後の`209.058 s`まで`-0.798975347 m`で完全に一定です。グラフ右端の水平線がこのhold状態です。

従ってゲートが防いだのは、VIOリセット後の`+0.928 m`や以後のVIO zをlocal odometryへ反映することです。出力自体を止めないのは、下流のTF・姿勢利用ノードがodometryを失わないようにするためです。roll/pitchはWit IMUから引き続き更新され、zだけが凍結されます。

一方で、グラフでおよそ145〜155秒に見える`+1.7 m -> -0.8 m`の下降は、閉鎖前に各フレームの変化が`max_z_step_m=0.25 m`未満だったため通過したVIO zのゆっくりしたドリフトです。現ゲートは「1フレームでのリセット」を検出できますが、数秒に渡る連続的なVIOドリフトは検出しません。これは現実装の限界です。

改善するなら、次段階でzの時間窓内変化量・z速度・wheel/IMUから期待される登坂量との整合性を追加し、ゆっくりした異常も隔離できます。ただし、実際の坂道・段差を誤って棄却しないよう、走行データを基に閾値を決めるべきです。
