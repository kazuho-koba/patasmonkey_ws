User prompt

いまこれでいうとステップ２まで終わったところですか？


ーーーーーーーーーーー以下過去にもらったサジェスト

1. `/odometry/gps` が0件になる原因を最優先で直す
   - `/fix`、IMU/heading、`/odometry/local`、`base_link ↔ gps_link` TFの成立を起動時に検査します。
   - 今回はIMU quaternion、共分散、GNSS fix、URDFのGPS固定TF自体には明白な欠落がありませんでした。`navsat_transform_node` は毎fixでDatumを読んでいますが変換出力を生成できていないため、次回はノードの購読先、TF lookup、変換初期化状態をliveで確認する必要があります。
   - 受入条件は `/fix` 約1 Hzに対して `/odometry/gps` も約1 Hzで継続し、0件なら走行前に異常扱いすることです。

2. local EKFからWit orientationの直接融合を外す
   - 現行local設定ではWitのroll/pitch/yawと角速度を全部使っています。[ekf_local_whl_imu_cam.yaml](/home/kazuho/Projects/patasmonkey_ws/src/pm_config/config/ekf_local_whl_imu_cam.yaml:32)
   - 以前の比較どおり、まずwheel vx＋Wit wzを基準とし、VIOは健全性判定後に速度を追加するのが安全です。
   - `wheel_odometry` にも `use_imu_initial_yaw: true` が残っているため、Wit orientationを完全に排除する条件ではfalseにします。[wheel_odometry.yaml](/home/kazuho/Projects/patasmonkey_ws/src/pm_config/config/wheel_odometry.yaml:21)
   - 平面走行を主目的にするなら、両EKFの`two_d_mode: true`と、GNSS altitude非融合を候補にします。

3. 地球基準headingをWit orientationから分離する
   - 現在のnavsat transformは `use_odometry_yaw: false` なので、`/wit/imu` orientationを直接使います。[navsat_transform.yaml](/home/kazuho/Projects/patasmonkey_ws/src/pm_config/config/navsat_transform.yaml:6)
   - local EKFからyawを外しても、ここを直さなければ既知の方位オフセット／歪みがGNSS座標変換へ残ります。
   - headingの優先順位は、検証済みのGNSS dual-antenna headingまたは`/navheading`、十分な速度で直進中のGNSS course-over-ground、較正済み磁気方位の順がよいです。停止中や低速時のcourseは使わず、速度閾値とヒステリスを設けます。
   - 信頼できる地球基準yawをodometryへ入れられる場合だけ `use_odometry_yaw: true` を使います。単に初期値0からwzを積分したyawは地球基準ではないので、そのまま切り替えてはいけません。

4. global EKFをGNSS拘束型にする
   - 第一候補は、globalにもwheel vx、Wit wz、健全なVIO速度を個別入力し、`/odometry/gps` はx/yだけ融合する並列構成です。
   - 最小変更案なら、`/odometry/local` を使い続けても、そのx/y/z poseはfalseにし、速度と必要最小限の姿勢変化だけを使用します。ただしlocal出力は複数センサを要約した相関済みデータなので、並列生入力より調整しにくいです。
   - 1 HzのGNSSに対してEKFを10～50 Hzで動かすこと自体は問題ありません。GNSS間は予測されます。global出力を制御に使わないなら10 Hzでも足りますが、表示・global planner向けに20～30 Hzへ上げても精度そのものは増えません。

5. GNSS品質ゲートをEKFの前に置く
   - RTK FIX/FLOATだけでなく、fix type、carrier solution、horizontal accuracy、衛星数、補正情報の鮮度、位置ジャンプ、local予測からのinnovationを使います。
   - NO FIXや不可能なジャンプは破棄し、低品質時は共分散を大きくします。再捕捉時にはヒステリシスを設けます。
   - 「non-fixなら最大10 m」と固定仮定するのは危険です。今回のbagでも水平σは中央値10.38 m、p95 17.50 mで、マルチパスや欺瞞では共分散自体が楽観的な場合もあります。
   - `odomN_pose_rejection_threshold`だけに依存せず、GNSS品質を明示した前処理ノードを置く方が診断しやすいです。

Codex final response

はい。大枠ではステップ2まで完了、次はステップ3です。

ただし正確には、ステップ2は「Wit orientationを全廃」ではなく、オフロード要件に合わせて次の形で完了しています。

- `separated_offroad`モードの水平EKFでは、Wit orientation（roll/pitch/yaw）を直接融合せず、wheel `vx`＋Wit `wz`＋健全性ゲート通過後のVIO `vx/vy`だけを使用します。
- Witのroll/pitchと、健全なVIO zは別の姿勢・高さ観測器で扱い、最終`/odometry/local`へ合成します。したがって坂・段差の姿勢と高さは維持しつつ、Wit yawやVIO z破綻が水平位置へ伝播しません。
- VIOが異常化すると、水平VIO速度は隔離され、高さは最後に検証済みのzで保持されます。
- 既存の`legacy`モードは変更せず残しています。新構成を実機で使うには`localization_mode:=separated_offroad`を明示します。

なお、提案中の`wheel_odometry.use_imu_initial_yaw`はまだ`true`のままです。ただし新しい水平EKFはwheel odometryの`vx`だけを読むため、これは新構成のEKF yawへは入っていません。Wheel odometry単体のpose/yawもWitから完全に独立させたい、という意味でfalse化するかは、単体wheel軌跡をどのように扱いたいか確認してから行うのが安全です。

ステップ1は、`navsat_transform_node`の実際の購読名に合わせて`("imu", "/wit/imu")`へ修正済みで、`/odometry/gps`が0件となった直接原因を解消しています。ただし実機での受入条件――`/fix`約1 Hzに対して`/odometry/gps`も継続して約1 Hz――の再確認は、次のGNSS接続実走時に行うべき状態です。

未着手は以下です。

3. `/navheading`などを使う地球基準headingの選定・navsat_transformへの投入
4. global EKFをGNSS拘束型に再設計
5. GNSS品質ゲートの実装

順序としては、ステップ3を先に行うのが適切です。navsat_transformが使用する地球基準yawを確定しないままステップ4へ進むと、GNSS位置変換の方位誤差をglobal EKFの調整で吸収してしまうためです。
