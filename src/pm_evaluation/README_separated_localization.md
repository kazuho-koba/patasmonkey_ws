# 分離localizerのbag再生と結果の読み方

`replay_separated_localization`は、記録済みbagからwheel、Wit IMU、VIOを再生し、
`separated_offroad`構成のlocalizationをオフライン評価します。実機のモータや
デバイスへ接続・制御することはありません。

## 生成物

出力先には、少なくとも次を保存します。

- `horizontal_ekf.csv`：wheel `vx`、Wit `wz`、gated VIO `vx/vy`の水平推定
- `attitude_height_observer.csv`：Wit roll/pitchと、gated VIO zまたは保持z
- `composed_local.csv`：従来互換の`/odometry/local`相当出力
- `vio_input.csv`：ゲート前のVIO入力
- `separated_localization_safe_output.png`：水平軌跡と姿勢・高さ観測器のz
- `summary.json`：サンプル数、期間、最終値、範囲
- log、再生MCAP、使用したEKF YAML

`attitude_height_observer.csv`のzが連続して出力されていても、それは毎IMU callback
で出力する仕様です。VIO高さゲートが閉鎖された後は最後に通過したzが一定になり、
新しいVIO zの追従を意味しません。

## 実行例

開発コンテナでFoxy、外部workspace、Patasmonkey workspaceを順にsourceして実行します。
具体的なbag指定と既存出力を上書きしない出力先指定は、CLIの`--help`を参照してください。

```bash
docker exec --user 1000:1000 --env HOME=/tmp patasmonkey_foxy_dev bash -lc '
  source /opt/ros/foxy/setup.bash
  source /workspaces/ros2_ws/install/setup.bash
  source /workspaces/patasmonkey_ws/install/setup.bash
  ros2 run pm_evaluation replay_separated_localization --help
'
```

7月・8月bagの既存結果は
`results/separated_offroad_localization/`以下にあります。これはVIO破綻の隔離性能を
調べるための再生結果であり、GNSS真値との差を直接表すものではありません。

GNSS拘束global EKF、heading初期化、品質ゲートを含む実走行評価では、bagに
`/fix`、`/fix/gated`、`/navpvt`、`/odometry/gps`、
`/gnss/fix_gate/diagnostics`も記録して比較してください。

## 磁気初期化を含むlocal/global再計算

`replay_global_localization`は元bagの`/wit/imu`、`/wit/mag`、車輪、
`/fix`、`/navpvt`、`/vio/odometry`を再生し、現在の補正ノード、停止判定、
VIO/GNSSゲート、local/global EKF、navsatを別ROS domainで実行します。
ハードウェアやモータ制御は起動しません。`--rate 1`は実時間相当です。
設定YAML・ログ・再計算bagは指定出力先へ保存し、既存ディレクトリは上書きしません。

```bash
ros2 run pm_evaluation replay_global_localization <入力bag> \
  --output-dir <新しい出力ディレクトリ> --rate 1
ros2 run pm_evaluation plot_global_localization_replay <出力ディレクトリ> \
  --basemap <同じ場所の既存basemap.npz>
```

描画は検証済み初期化後の軌跡を使い、local/globalには方位fitを行いません。
VIOは独自の原点・向きを持つため初期yawだけをlocalに合わせます。VIO自体は
記録済み`/vio/odometry`で、OpenVINSの再実行ではありません。
地図背景は指定された同一地点のOSMキャッシュを使い、GNSSとの距離差は真値誤差
ではなく参考比較です。別の試行から得た磁気校正値を古いbagに適用する場合、
その期間の車体構成・磁気環境に適合する保証はありません。
診断時は`audit_global_replay <出力ディレクトリ>`と
`plot_global_replay_audit <出力ディレクトリ>`でglobal状態・入力のCSVと図を
作れます。navsatの参照を比較する`--navsat-odom-topic /odometry/global`は
比較用の引数で、本番設定を変更しません。既定は現在の本番と同じglobal基準です。

`analyze_localization_tf <出力ディレクトリ>`は、再生時に保存した`/tf`と
GNSS・local・globalを近い時刻で照合し、座標変換のCSV、集計と図を作成します。
記録後の出力値の比較であり、EKF内部の更新前innovationではありません。

既定は`--global-separated --navsat-odom-topic /odometry/global`で、本番と同じ
global水平推定にlocalのroll・pitch・相対zを合成します。高度は地球基準でなく、
VIO高さゲートが閉じた後は保持値です。時刻差0.2秒を超える場合は合成しません。
`--global-3d`は過去の3D方式との比較用、`--navsat-odom-topic /odometry/local`は
過去の座標循環の再現用であり、いずれも実機での使用を推奨しません。
`--local-ekf-config <yaml>`では本番設定を変更せず、異なるprocess noise等の
local EKF設定をbag再生で比較できます。設定は出力先に保存されます。
小さい共分散の出力は推定精度の証明ではなく、実走行で滑り・横移動を検証してください。
