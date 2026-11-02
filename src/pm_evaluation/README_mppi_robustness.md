# MPPI制約整合・遅延とモデル誤差の机上検証

`tools/mppi_robustness.py`は共通の`pm_config/config/vehicle_control.yaml`と同じディレクトリの
`vehicle_geometry.yaml`を読み、指令制約の整合と人工閉ループ追従を評価する。
Foxyのoverlayをsourceしたコンテナで実行する。ROS node、driver、motorは起動しない。

```bash
python3 src/pm_evaluation/tools/mppi_robustness.py --output /tmp/mppi_robustness_new
python3 src/pm_evaluation/tools/mppi_robustness.py --target-speed 1.0 --output /tmp/mppi_1mps_new
python3 src/pm_evaluation/tools/mppi_robustness.py --target-speed 1.0 --constant-speed-reference --output /tmp/mppi_old_reference
```

出力先は未作成のディレクトリを指定する。`--workspace`でworkspaceを変更でき、
単独試験の別設定は`--vehicle-config /absolute/path/vehicle_control.yaml`で指定する。
その隣のgeometryがなければvehicle interfaceの寸法宣言既定値を用いる。

通常のplannerも同じ共通YAMLの車体yaw上限・加速度をそのまま読み、`vehicle_config` parameterで
単独試験の別ファイルを指定できる。旋回効率はinterfaceで輪差へ変換するときに使い、
planner側では上限の係数倍や半径の拡大を行わない。

評価器はvehicle_interface_nodeの制限・車輪変換の数値メソッドをASTで抽出し、
モータ通信に到達しないオブジェクトで実行する。制約式を別途模写して一致とみなすことを避ける。
MPPIの計算と車体の間にこの実際の制限処理を挟み、指令遅延、観測遅延、旋回係数の±20%、
位置・姿勢ノイズ、周期変動、初期位置・向きの誤差を与える。
評価器の終点停止はゼロ要求を継続し、実速度が停止条件へ落ちるまで待つ。
GUI・実行管理の停止連携や通信断検出はこのツールでは検証しない。

`audit.json`に実効制約、`results.json`に到達・waypoint状態・最大/95%Path誤差を保存。
各ケースのCSVとSVGは指令・真の車体軌跡・参照経路を示す。`moving_stop_probe`は
走行5秒以降の状態を複製してゼロ要求を出した後の人工モデルの停止距離・時間。
終点到達後の停止距離も別フィールドにあるが、ほぼ停止済みなので混同しない。

半径の余裕は0.1 m、MPPIのseedは42。目標速度は`--target-speed`で指定し、既定0.15 m/s。
参照速度は曲率方式が既定で、`--constant-speed-reference`は従来方式との比較用。
ROS launch側の目標速度既定1 m/sと、評価器の互換用既定0.15 m/sは異なる。
目標速度を増やしても旋回制限は緩めない。CSVは要求速度・interface適用後速度・
人工車体の実速度を別々に記録する。実際の速度が目標に常時一致するものではない。
現在の評価器は共有車両YAMLを用いるが、planner.yamlの余裕など他の全設定を
自動で読み込む汎用runnerではない。各設定はスクリプトとconfiguration.jsonを参照する。
時間は仮想時間で、処理を実時間10 Hzでスケジューリングした試験ではない。
人工モデルの結果で実車のタイヤ滑り、路面・モータ応答、Jetsonの負荷を証明することはできない。
