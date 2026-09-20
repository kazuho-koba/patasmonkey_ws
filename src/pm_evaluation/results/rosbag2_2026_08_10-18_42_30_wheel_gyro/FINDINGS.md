# rosbag2_2026_08_10-18_42_30: wheel vx + Wit wz

## 実行条件

- EKF: Foxy `robot_localization/ekf_node`, `source_wheel_gyro.yaml`
- 入力: `/wheel/odometry` の `vx` と `/wit/imu` の `wz` のみ
- VIO: 読み込み、再生、融合、評価のいずれにも不使用
- 比較共通区間: 234.936 s
- 初期位置のみGNSSへ合わせ、scaleや軌跡全体の位置合わせは行わない

## 軌跡と角度補正

既存の表示補正 `+90 deg` ではGNSSとの方位が合わず、GNSS不一致RMSEは
36.63 m、終点差は66.98 mだった。

GNSS ENU速度ベクトルとEKF速度方向を重み付きで合わせた補正角は、公開済み
`/wit/imu` yawへ加える角度として `+137.708 deg`。速度・共分散閾値を変えた
感度範囲は `+136.712 ... +137.894 deg`、単純bootstrap 95%区間は
`+136.240 ... +139.299 deg` だった。

位置差分による推定は `+139.852 ... +141.068 deg`、全位置のProcrustes推定は
`+140.561 deg`。wheel入力の単発異常の影響を受けにくいGNSS速度推定を主結果とし、
このbagの暫定表示補正には `+137.7 deg` を採用する。

`+137.708 deg`で再描画した結果、GNSS不一致RMSEは2.16 m、終点差は2.09 m。
GNSS時刻での誤差はmedian 1.76 m、p95 3.58 m、最大4.91 mであり、角度補正後の
大域的な軌跡形状は十分一致している。

## 理論上の補正

- ROS/map規約はENUで、yaw 0が東、反時計回り正。
- URDFの`base_link -> wit_imu_link` yawは0 deg。
- 現行HWT905ドライバはセンサyawから90 degを引いて公開する。
- 従って、ドライバが正しいROS ENU yawを公開する設計ならplot側の補正は0 deg。
- 現状のplot `+90 deg`はドライバの`-90 deg`を相殺して生センサyawへ戻す値であり、
  真北に合わせる校正値ではない。

今回のGNSSフィット `+137.7 deg` は、ドライバ相殺分 `+90 deg` に加えて約
`+47.7 deg`を必要とする。前bagの約`+119.7 deg`とも一致しないため、現時点では
固定定数を全bagへ適用せず、Witの方位基準、磁気偏角、車載磁気外乱、起動時校正を
切り分ける必要がある。

## 発見したwheel入力異常

約3.457 sのwheel topic/header gap後、`vx=-11.319 m/s`の単発外れ値があり、
EKF軌跡に最大18.08 mの瞬間的な位置ジャンプが生じた。直前のvxはほぼ0、直後も
ほぼ0であり、同区間のGNSS速度は概ね1 m/s未満なので実移動ではない。
EKF poseの0.2 m超の高レート増分は7件だけで、この異常に集中している。

高レートEKF軌跡長130.72 mに対しGNSS 1 Hz折れ線長99.55 mとなる差の主因でもある。
角度推定は速度品質フィルタと複数手法の一致で確認したが、このwheel生成側の
timestamp gap処理は別途修正対象である。

## GNSS品質とnavheading

- NAV-PVT分類: GNSS 234件、RTK_FLOAT 55件、RTK_FIX 0件
- 記録covarianceからの水平sigma中央値: 7.71 m

`/navheading`は車体heading検証には使っていない。対応する`/navrelposned`の基線長が
約5.5 kmであり、これは基地局からroverへの相対位置ベクトルのheadingで、車体上の
2アンテナheadingではないためである。
