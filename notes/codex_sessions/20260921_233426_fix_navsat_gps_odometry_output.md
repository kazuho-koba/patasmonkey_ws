User prompt

1. /odometry/gps が0件になる原因を最優先で直す   の実行をお願いします。
実機試験が必要であれば、jetsonの方に全てつながっているのでそちらで実施してください。
コードレビューだけで足りるなら、開発PCの開発コンテナ内で実施してください。

Codex final response

`/odometry/gps` が0件だった原因を修正しました。実機試験は不要で、既存bagを用いたFoxyコンテナ内の再生試験で原因特定と修正確認まで完了しています。

原因はROS 2 Foxy版 `robot_localization` の `navsat_transform_node` がIMUを相対トピック名 `imu` で購読することでした。従来launchは `imu/data → /wit/imu` をremapしていたため、navsat_transformは`/wit/imu`を一度も受信できず、変換初期化に必要なIMU条件が満たされませんでした。その結果、`/fix`を受信してDatumログを出していても`/odometry/gps`をpublishしていませんでした。

修正したファイルは次の4つです。

- `src/pm_bringup/launch/pm_bag_global_localization.launch.py`
- `src/pm_bringup/launch/pm_global_localization.launch.py`
- `src/pm_bringup/launch/pm_bag_shibetsu.launch.py`
- `src/pm_bringup/launch/pm_imu_test.launch.py`

すべてのnavsat remapを次のように変更しました。

```python
("imu", "/wit/imu")
```

検証では、7月26日のbagから`/fix`、`/wit/imu`、`/odometry/local`、TFをread-only再生しました。

- 旧remap `imu/data → /wit/imu`: `/odometry/gps` は0件
- 新remap `imu → /wit/imu`: 20秒の再生で`/odometry/gps`を34件受信

新remap時のログには、navsat_transformがIMUを使って変換初期化を完了したことを示すheading補正ログも出ています。これにより、原因はTF・GNSS fix・URDFではなくFoxyの購読名との不一致であることを再現性をもって確認できました。

`pm_bringup` はFoxy開発コンテナでビルド済みです。

```text
Finished <<< pm_bringup
Summary: 1 package finished
```

今回起動したbag再生・navsat・静的TFの試験プロセスはすべて停止済みです。

Jetsonへ反映後、次の実走行では`/fix`が約1 Hzなら`/odometry/gps`も継続して出ることを確認してください。bagには既に`/odometry/gps`が記録対象として入っているため、新しいbagの`topic_frequency.csv`で即座に検証できます。
