User prompt
ステップ１〜５で実施した内容を改めて解説してもらえませんか？
水平方向とｚ方向のオドメトリ推定の分離、GNSS拘束については、patasmonkey_ws/notes/codex_sessionsにファイルを作って図示もしてほしいです。

Codex final response
ステップ1〜5の設計・実装内容を、図付きの設計ノートとしてまとめました。

[localization_steps_1_to_5_design.md](/home/kazuho/Projects/patasmonkey_ws/notes/codex_sessions/localization_steps_1_to_5_design.md) に、全体のデータフロー、水平方向と姿勢・zの分離、heading初期化、GNSS拘束global EKF、GNSS品質ゲートをMermaid図で記載しています。

要点は次のとおりです。

1. **`/odometry/gps`の復旧**: Foxy版`navsat_transform_node`の実際のIMU購読名に合わせ、`imu -> /wit/imu`へ修正しました。7月bag再生で0件から34件へ復旧を確認済みです。
2. **localの役割分離**: 水平EKFはwheel `vx`、Wit `wz`、健全なVIO `vx/vy`だけを用い、roll/pitch/zは別の観測器で扱います。VIO破綻がx/y/yawやz全体を破壊しない構成です。
3. **地球基準headingの初期化**: 停止中のWit yawを一度だけEKFへ投入し、その後のyawは`wz`で伝播します。校正済みの固定補正値を入れる場所は用意済みですが、値はまだ未確定です。
4. **GNSS拘束型global EKF**: `/odometry/local`のposeはglobalへ渡さず、生wheel/VIO速度・Wit姿勢/`wz`で20 Hz予測し、GNSSのmap `x/y`を約1 Hzの絶対位置補正として使います。
5. **GNSS品質ゲート**: `/fix`と`/navpvt`からfix状態、衛星数、`h_acc`、RTK状態、位置ジャンプを検査して`/fix/gated`を生成します。non-fixを一律棄却せず、共分散を大きくして弱い観測として扱います。

実機では、`localization_mode:=separated_offroad`を指定して使用します。まず`/fix`、`/fix/gated`、`/odometry/gps`が初期3サンプル後に約1 Hzで揃うことと、`/gnss/fix_gate/diagnostics`の採否理由を確認してください。
