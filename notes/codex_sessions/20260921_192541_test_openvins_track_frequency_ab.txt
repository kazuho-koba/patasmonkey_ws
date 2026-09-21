【ユーザープロンプト】
ありがとうございます、ではまずtrack_frequencyの値修正によるA/Bテストを試してください。
着手前に、track_frequencyはそもそもどこに書いてあるパラメータ７日も教えてください。

【Codex最終回答】
A/Bテストの結果、`track_frequency: 21.0`で画像更新レートは13.51 Hzから19.37 Hzへ回復しました。今回のデータでは、処理遅延や追跡破綻が増えた兆候はありません。

`track_frequency`はOpenVINSの特徴追跡上限レートで、Patasmonkey側では [estimator_config1.yaml](/home/kazuho/Projects/patasmonkey_ws/src/pm_config/config/oak_d_s2/estimator_config1.yaml:87) にあります。

```yaml
track_frequency: 20.0
```

この設定ファイルを [pm_bag_global_localization.launch.py](/home/kazuho/Projects/patasmonkey_ws/src/pm_bringup/launch/pm_bag_global_localization.launch.py:63) がOpenVINSへ渡しています。実際の間引き判定は [ROS2Visualizer.cpp](/home/kazuho/ros2_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.cpp:540) にあります。

比較には、手動jerkを含む同一bagを使用しました。

| 指標 | A: 20 Hz | B: 21 Hz |
|---|---:|---:|
| 状態更新数 | 2,136 | 3,066 |
| 画像更新レート | 13.51 Hz | 19.37 Hz |
| 平均推定処理時間 | 17.72 ms | 10.87 ms |
| p95 | 25.89 ms | 14.55 ms |
| 最大 | 101.32 ms | 21.05 ms |
| 50 ms超過 | 7件 | 0件 |
| 正味変位 | 0.173 m | 0.145 m |
| 推定軌跡長 | 0.856 m | 0.623 m |

21 Hz側ではOpenVINSが正常に初期化され、追跡喪失、キュー飽和、NaN、異常終了を示すログは0件でした。更新数は約43.5%増え、20 Hz画像入力にほぼ追従しています。

ただし、Aは元の実機オンライン記録、BはそのbagをOpenVINS単体へ実時間再生した結果です。したがって処理時間の短縮には、Bで他ノードの負荷が少ない効果も含まれます。処理時間そのものを「21 Hzの方が速い」とは判断できません。一方、同一の画像・IMU timestampに対して更新レートが回復したことと、21 Hz側が50 ms周期内で処理できたことは確認できます。

この試行は車体がほぼ静止した短時間データなので、絶対精度や走行中の破綻率までは判定できません。正味変位と軌跡長は21 Hz側の方が小さく、少なくとも静止ドリフトが悪化した兆候はありません。

有効なB結果はJetson上の次の場所です。

`/home/nvidia/patasmonkey_ws/bags/openvins_ab_track_frequency_20260921_21hz_retry2`

- OpenVINS state/timing/log: `openvins/`
- 出力bag: `outputs/`（4.8 GiB、34,410メッセージ）

途中の2回は、MCAP形式の自動判別失敗と設定ファイルの相対参照切れにより出力0件でした。上書きや削除はせず、小さな失敗記録として以下に残しています。

- `openvins_ab_track_frequency_20260921_21hz`
- `openvins_ab_track_frequency_20260921_21hz_retry`

結論として、`21.0`は今回の20 Hz入力を正常に通す暫定値として妥当です。ただしリポジトリおよびJetsonの本番設定はまだ`20.0`のままです。次は本番設定を21 Hzへ変更し、実走行で入力数、CPU負荷、特徴点数、軌跡精度、破綻有無を再確認する段階です。
