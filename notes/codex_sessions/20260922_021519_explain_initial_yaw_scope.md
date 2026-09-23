User prompt

IMUのyaw角を使うのは、localとglobalの両方のオドメトリや姿勢推定の初期化に使う、ということですか？

Codex final response

「同じ校正済みheadingを、localとglobalの整合のために一度だけ共有する」は正しいですが、両EKFへIMU yawを別々に継続融合する、という意味ではありません。

推奨する役割分担は次です。

| 箇所 | 校正済みIMU yawの使い方 |
|---|---|
| local EKF | 起動時または明示リセット時に1回だけyaw初期値として設定。その後は`wz`で伝播し、磁気yawは直接融合しない。 |
| navsat_transform | datum確立時に地球基準headingとして1回使う。GNSS緯度経度をローカル/地図座標へ正しく回転するための基準。 |
| global EKF | IMU yawを追加で直接融合しない。localの相対姿勢変化と`/odometry/gps`の位置を融合する。 |

つまり、global EKFの初期yawを別途IMUから二重に与える必要はありません。globalはlocalからyawの連続性を受け取り、navsat_transformが校正済みの初期headingでGNSS座標の向きを決める、という関係です。

現行の[ekf_global_3d.yaml](/home/kazuho/Projects/patasmonkey_ws/src/pm_config/config/ekf_global_3d.yaml)も、`/odometry/local`については姿勢を含めて融合し、`/odometry/gps`についてはx/y/z位置だけを融合しています。このためglobalへ`/wit/imu`を追加すると、local経由のyawと相関した同じセンサ情報を二重計上する可能性があります。

一点だけ実装上の重要な補足があります。現在の`separated_offroad`水平EKFはyawをゼロから開始して`wz`積分するため、「localにも初期yawを与える」には、校正済みheadingを1回だけ`set_pose`/初期状態として渡す経路を追加する必要があります。wheel odometryの`use_imu_initial_yaw`は現在trueですが、新しい水平EKFはwheelの`vx`だけを読むので、これだけでは水平EKFのyaw初期値にはなりません。

実装では校正済みheadingを一か所で生成し、同じ値を次の二用途へ配ります。

1. 水平local EKFの一回限りの初期yaw
2. navsat_transformのdatum時のheading

同じ物理的headingを二重に「補正」しないことが重要です。Witの磁北/真北変換、センサ取付角、ROS ENU座標系の規約を一つの前処理で正規化してから渡します。特にrobot_localizationのyaw規約は東向き0 rad・反時計回り正であり、一般的な磁気コンパスの北向き0度・時計回り正とは異なり得るため、`yaw_offset`の符号を実測で検証します。

したがって、答えは「localには初期値として、globalにはnavsat_transformを介した地球基準の座標整合として使う。global EKF自身には直接追加しない」です。
