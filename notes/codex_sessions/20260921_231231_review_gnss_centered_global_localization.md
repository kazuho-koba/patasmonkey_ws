User prompt

ありがとうございました、コードの分析・修正はいったんオシマイにして、改めて実走行データをとってから改めて検証します。
  改めてpatasmonkey_ws/bags/rosbag2_2026_07_26-09_17_38　など見ると、global ekfの結果がかなりlocal ekfの結果に引っ張られています。
  GNSSが明らかに妨害や欺瞞を受けていたり、数十mオーダーでの誤差が見込まれているならともかく、RTKがnon-fixでも誤差がせいぜい十m程度なら、globalの自己位置推定はある程度はGNSS情報をベースに考えて、local ekf
  の結果は1Hzでしか出ないGNSS情報の間を補完するような使い方をしたいです。
 
  以前やったlocal ekfの入力ソース調整（IMUのorientationは直接使わないなど）も必要に応じて考慮しつつ、上記の方針の是非について検討してください。もっといい方法があればそれを提案してください。
  そして良いと思われる方法を実現する具体的な実装方針にどのようなものがありそうか検討してください。

Codex final response

方針は妥当です。Patasmonkeyでは「localは連続で滑らかな相対運動」「globalはGNSSで絶対位置を拘束し、その間をデッドレコニングで予測」という二段構成が適しています。ただし、今回のbagについては重み調整以前の問題が見つかりました。

`rosbag2_2026_07_26-09_17_38` には `/fix` が229件ある一方、`/odometry/gps` は0件です。7月10日～8月22日の手元bagも、確認できた全試行で `/odometry/gps` が0件でした。そのため、このbagのglobal EKFはGNSS位置を弱く融合していたのではなく、実質的にGNSS位置を一度も受け取らず、local EKFだけで動作していたと考えるべきです。

現行global設定は `/odometry/local` の位置・姿勢・速度をほぼ全部、10 Hzのglobal EKFへ絶対観測として投入し、GNSS位置も投入する設計です。[ekf_global_3d.yaml](/home/kazuho/Projects/patasmonkey_ws/src/pm_config/config/ekf_global_3d.yaml:14) を見ると、localのx/y/z、roll/pitch/yaw、各速度がすべて有効です。仮に `/odometry/gps` を復旧しても、50 Hzの相関したlocal poseを繰り返し入れるため、globalが過信してGNSS補正を弱くする可能性があります。

実データでも、localの位置分散中央値はxが85.5 m²、yが588 m²なのに、globalはxが0.106 m²、yが0.797 m²まで小さくなっています。一方、`/fix` の水平合成σ中央値は10.38 m、p95は17.50 mです。この状態ではglobal側が不自然に自信過剰です。

推奨する構成は、robot_localizationで一般的な「並列dual EKF」です。

| 系統 | 役割 | 主な入力 | 出力 |
|---|---|---|---|
| local EKF | 短時間で滑らかな相対移動 | wheel vx、Wit wz、健全時のみVIO | `odom → base_link`、`/odometry/local` |
| navsat transform | 緯度経度を走行座標へ変換 | `/fix`、信頼できる地球基準heading、local odometry | `/odometry/gps` |
| global EKF | GNSSで絶対位置を拘束し、GNSS間を予測 | wheel/IMU/VIOの連続量＋GNSS x/y | `map → odom`、`/odometry/global` |

global EKFにはlocalの絶対x/y poseを入れず、localと同じ生の連続センサを並列に入れるのが第一候補です。これにより、GNSS更新間はwheel・IMU・VIOで予測し、1 HzのGNSS到着時に絶対位置を補正できます。これは厳密には「補間」ではなくEKFの予測です。制御はジャンプしないlocal odometryを使い、地図上の自己位置やglobal plannerはglobalを使います。

実装優先順位は次のとおりです。

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
   - headingの優先順位は、検証済みのGNSS dual-antenna headingまたは`/navheading`、十分な速度で直進中のGNSS course-over-ground、較正済み磁気方位の順がよいです。停止中や低速時のcourseは使わず、速度閾値とヒステリシスを設けます。
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

GNSSをさらに強く基準にしたい場合は、global EKFとは別に`map → odom`補正量を管理する方式もあります。GNSS到着時に「GNSS位置－local予測位置」の残差でmap/odom変換を更新し、品質に応じて即時補正または速度制限付き補正を行います。要求との対応は明快ですが、まずは標準的なdual EKFを正しく成立させる方が保守しやすいです。遅延GNSS、ロバスト損失、過去状態の再最適化まで必要になれば、その次にfixed-lag smootherやfactor graphを検討する順番がよいです。

次の実走行では、少なくとも `/fix`、`/odometry/gps`、`/odometry/local`、`/odometry/global`、`/tf`、GNSS品質トピックを記録し、GNSS更新ごとのinnovationとKalman補正量を比較してください。A/B条件は「local poseをglobalへ全投入」「localのtwistだけ＋GNSS」「並列の生センサ＋GNSS」の3条件が有効です。

結論として、目指す考え方は正しいです。ただし最初に直すべきなのは重みではなく、長期間0件になっている `/odometry/gps` の生成経路です。その後、globalへのlocal絶対pose投入をやめ、Wit orientationをheading系からも切り離したdual EKFへ移るのが、最も筋のよい実装方針です。
