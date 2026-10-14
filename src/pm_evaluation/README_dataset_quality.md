# データセットの保存品質の後処理

`analyze_dataset_quality` は既存の `analyze_bag_frequencies` とは別のCLIです。
MCAP形式の1つのbagを読み、時間窓ごとにレート・欠測・遅延・左右対応を検査します。
bag原本は削除・変更せず、ROSノードや機体も起動しません。

## 実行

Foxy環境で `pm_evaluation` をbuildし、overlayをsourceしてください。

```bash
ros2 run pm_evaluation analyze_dataset_quality /path/to/bag \
  --policy /path/to/patasmonkey_ws/src/pm_evaluation/config/dataset_quality.yaml
```

ROS CLIなしでソースから実行することもできます。Python依存は
`mcap`、`PyYAML`、`matplotlib`です。既存周波数CLIと同じMCAP依存を使います。

```bash
python3 src/pm_evaluation/pm_evaluation/cli/analyze_dataset_quality.py \
  /path/to/bag --policy src/pm_evaluation/config/dataset_quality.yaml
```

既定の出力先はbag内 `quality/`。再解析では `--output /path/to/quality_v2`
を指定します。既存ディレクトリの上書きは禁止し、原本と以前の判定を保護します。
計測終了後、recorderが停止してmetadataが確定したbagに実行してください。
停止処理から自動実行する機能は今回追加していません。

## 保存ファイル

- `policy.yaml`: 実際に使った判定設定。
- `summary.yaml`: 元bag識別情報、解析日時、各トピックの初回・最終時刻、
  全体／トピック別の暫定有効区間、安定開始・確認時刻、失敗理由。
- `topic_windows.csv`: 各窓のrecord/Header件数Hz、Header間隔Hz、最大間隔・遅延、合否。
- `timeline.png`: 想定Hz比と必須条件の合否。図の英語軸名はフォント依存を避けるため。
- `completed.json`: 全出力生成後の完了マーカー。これがない出力は完成扱いしない。

解析正常終了は終了コード0です。これは品質合格を意味しません。
合格区間がなければsummaryは `no_qualified_interval`、開始時刻はnullです。
周波数だけの判定は `rate_only_stable_start_sec` と
`rate_only_valid_intervals` に別保存します。これは欠測間隔・左右対応・遅延の
合格を意味しません。追加条件を含む判定は `stable_start_sec` です。
入力欠損・不正設定・既存出力先などは終了コード2になります。

## 時刻と判定

基準はmetadataのbag開始時刻。Headerも同じ絶対時刻系から相対秒へ変換し、
トピックごとに独立してゼロへ揃え直しません。
既定では10秒窓を1秒ずつ移動し、窓全体の件数÷10秒を想定Hzと比較します。
最初のメッセージまでの欠測も検査するため、初期窓は短縮しません。
窓内の最初〜最後で割るHeader間隔Hzは診断用で、レート判定には使いません。
末尾の10秒窓を構成できない端数部分は未評価とし、その開始時刻を記録します。

連続3窓が合格したrunを暫定利用可能区間として保存します。10秒窓・1秒刻み
では確認に12秒かかります。30秒の独立観測ではありません。
開始時刻はrunの最初の窓の先頭、確認時刻は3番目の窓の末尾です。
途中で失敗した場合はrunを分け、単一の開始時刻だけで全後半を有効としません。
重複する窓に基づく暫定区間なので、境界付近の個別サンプルの無異常は保証しません。

`clock: record` はMCAP記録時刻、`header` はHeader時刻で件数・間隔を判定します。
両件数Hzは常に併記します。Headerの重複・逆行、不正値、未来時刻、記録遅延も検査します。
Headerは標準ROS型の先頭CDRだけを読みます。独自型は `header: false` でrecord評価するか、
型の確認とデコーダ拡張が必要です。デバイス内部timestampを読み取る機能ではありません。
未来Header判定の許容は `max_future_sec`（既定0.001秒）で変更できます。

## 設定の変更

`config/dataset_quality.yaml` は暫定基準です。`required: true` の全トピックと
必須stereo条件が合格した窓を全体合格とします。optionalは個別結果のみで全体を拘束しません。
既定でWitと左右モノクロ画像が必須です。VO用の完全な入力品質認証ではありません。
OAK IMUは出力仕様の確定前のためexpected_hzがnullでoptionalです。
VO向けの運用基準には、その想定Hzを確認してrequiredへ変更してください。
未設定Hzは自動合格にせず `expected_hz_unconfigured` と出力します。

`tolerance_ratio: 0.01` は±1%。`max_gap_sec` と `max_delay_sec` は秒です。
低レートGNSSは整数件数の影響が大きいため `rate_check: false` とし、
最大間隔で確認しています。GNSS fix品質や精度は別の解析が必要です。
窓境界をまたぐ欠測も評価します。左右画像はHeader時刻の1対1対応率で確認し、
sequence番号や厳密な画像–IMU同期は評価しません。

用途に応じて必須トピックと閾値を明示してください。E2E学習の操作指令・車体状態を
追加する場合は、そのROS型・Header有無・実際の周期を確認して登録します。
既定の閾値は初期設定であり、今回の2bagで安全性・学習適合性を実証した値ではありません。
