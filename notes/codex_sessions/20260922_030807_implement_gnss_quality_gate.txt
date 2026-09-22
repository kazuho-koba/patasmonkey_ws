User prompt
ありがとうございます、ではそれをお願いします。

Codex final response
ステップ5のGNSS品質ゲートを実装しました。`localization_mode:=separated_offroad` のときだけ有効で、既定のlegacy経路は変更していません。

新しい [gnss_fix_gate_node.py](/home/kazuho/Projects/patasmonkey_ws/src/pm_localization/pm_localization/gnss_fix_gate_node.py:1) は、生の`/fix`とu-bloxの`/navpvt`を照合し、採用したものだけを`/fix/gated`へpublishします。`navsat_transform`と初期datum設定は、このgated streamだけを使います。

判定内容は以下です。

- `NavSatFix`がNO_FIXでないこと、緯度経度が有限値であること
- `/navpvt`が2.5秒以内で、3D fix・`GNSS_FIX_OK`・衛星数6以上であること
- receiverの`h_acc`が20 m以下であること
- 直前候補からの移動が、5 m/s + 3 mの余裕を超えないこと
- 起動直後・棄却後は、3回連続の正常fixを確認してから再採用すること

RTK FIXだけに限定してはいません。`NAV-PVT`のcarrier-phase状態に応じて、水平1σ共分散の下限をRTK FIX=0.5 m、FLOAT=2 m、standalone/DGNSS=10 mとして、receiver報告の`h_acc`および元の`NavSatFix`共分散のうち最も大きい値を使います。このため、non-fixも品質が許容範囲なら弱い位置観測として利用し、global EKFを過度に引っ張りません。

設定は [gnss_fix_gate.yaml](/home/kazuho/Projects/patasmonkey_ws/src/pm_config/config/gnss_fix_gate.yaml:1) に分離しました。走行後は、追加した`/gnss/fix_gate/diagnostics`、生の`/fix`、`/fix/gated`、`/navpvt`を同じbagへ記録します。診断には採否、棄却理由、RTK状態、衛星数、`h_acc`、連続正常fix数が残ります。

[pm_bag_global_localization.launch.py](/home/kazuho/Projects/patasmonkey_ws/src/pm_bringup/launch/pm_bag_global_localization.launch.py:412) と [pm_global_localization.launch.py](/home/kazuho/Projects/patasmonkey_ws/src/pm_bringup/launch/pm_global_localization.launch.py:208) に組み込み済みです。実行は従来どおりです。

```bash
ros2 launch pm_bringup pm_bag_global_localization.launch.py \\
  localization_mode:=separated_offroad
```

Docker開発環境で `pm_config`、`pm_localization`、`pm_bringup`、`pm_evaluation` をビルドし、両launchの構文確認まで完了しています。実機ではまず次を確認してください。

```bash
ros2 topic hz /fix /fix/gated /odometry/gps
ros2 topic echo --once /gnss/fix_gate/diagnostics
```

期待値は、正常な1 Hz GNSSで初回3サンプル後に`/fix/gated`と`/odometry/gps`が約1 Hzとなることです。`/fix`だけが出る場合は、診断トピックの棄却理由を基に閾値を調整します。
