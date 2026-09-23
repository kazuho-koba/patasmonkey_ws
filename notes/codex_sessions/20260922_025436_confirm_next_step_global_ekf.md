User prompt

実機確認はできないので、次はステップ4ですかね？

Codex final response

はい。次はステップ4、global EKFをGNSS拘束型へ再設計する段階です。

実機校正が保留でも進められます。ステップ3で作ったheading初期化は、校正値が確定するまで`yaw_correction_radians: 0.0`のまま保持し、ステップ4ではその値へ依存せずに「global位置がlocal EKFのposeを過信しない」構造へ直します。

現在のglobal EKFは`/odometry/local`のx/y/z・姿勢・速度をすべて融合し、`/odometry/gps`は位置だけを融合しています。そのためlocalの位置ドリフトやVIO由来の誤差がglobalへ強く伝わります。

推奨するステップ4の実装方針は以下です。

1. global EKFへ`/odometry/local`のpose（x/y/z）を入れない。
2. global EKFの予測入力は、wheel `vx`、Wit `wz`、健全時だけのVIO `vx/vy`を個別に入力する。VIOはlocalと同じ速度ゲート後のtopicを使う。
3. `/odometry/gps`はglobalのx/y位置観測として融合する。zはGNSS品質を踏まえて当面除外する候補です。
4. globalのroll/pitch/yawは、localから相関済みのposeを複製せず、必要最小限の相対姿勢変化として扱う。地球基準yawの絶対拘束は、ステップ3の実機校正完了後に別途追加します。

これにより、GNSSが約1 Hzでも、GNSS間をwheel/IMU/VIO速度で補間しながら、数十m級のlocal位置誤差をglobalへそのまま持ち込まない構成になります。

次の作業では、現行global YAML・navsat出力の共分散・7月/8月bagを確認して、候補YAMLを作成し、再生MCAP・CSV・GNSS/OSM重畳プロットで現行構成と比較します。
