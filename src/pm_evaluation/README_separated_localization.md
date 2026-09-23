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
