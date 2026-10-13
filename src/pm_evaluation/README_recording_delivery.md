# IMU配送とbag記録の切り分け

## 分割bagのオフライン再生

`pm_evaluation/cli/bag_clock_player.py`は選択したセンサtopicと`/clock`を
記録時刻に沿って配信します。分割MCAPはファイル名の連番を数値順に読むため、
10分割を超えても`bag_10`が`bag_2`より先に再生される時刻逆行を防ぎます。
元messageのheaderやpayloadは変更しません。センサ時刻を基準とする比較には、
log_timeだけをheader stampへ置換した派生MCAPを別ディレクトリに作成してください。
車両制御topicは再生対象に含めず、実機と別のROS_DOMAIN_IDで実行してください。

## シンプルなGUI有無比較を最初に行う

最初はSSD上の単一bag・通常writer・追加recorderなしで比較します。
RAM保存や非同期writerは、通常構成で達成できる性能と限界を確認した後の候補です。
同じcoreとrecorderを維持し、GUIなし→GUIのカメラOFF→ON→GUIなしのeventを
wall_time付きJSONLに保存すると、以下で全topicの定常記録率を比較できます。
PCとJetsonの時計同期が必要です。解析は停止・metadata確定後に行ってください。

```bash
python3 src/pm_evaluation/tools/compare_gui_stages.py \
  --bag /path/to/stopped_bag --events /path/to/gui_events.jsonl \
  --output /path/to/new_stage_analysis
```

VIO有効の性能試験では、例えば`--require-topic /ov_msckf/odomimu:100`を追加すると、
どれかの定常区間が最低100 Hz未満の場合に終了code 1となります。
解析CSV/JSONは保存され、requirements.jsonに未達条件が残ります。
100 Hzはこの装置の約127 Hz出力を確認する評価用の例で、OpenVINSへ設定する値ではありません。
必要な全topicの期待値に応じて複数回指定してください。

eventは`event`（begin/end/camera_off_end/camera_on_begin）、`mode`、`wall_time`
（UNIX秒）を含めます。mode=offonのbeginはカメラOFF、camera_on_beginはONです。
区間先頭3秒・末尾2秒を除き、CSV/JSONに件数、固定区間Hz、header周期、最大記録空白を
残します。GUIのJPEG表示枚数とは別の値です。独立プローブを追加しないため、
全publish件数に対する欠測率の保証はできません。短時間の好結果だけで長期保証とはしません。

固定sleepやrecord_trial wrapperの生存だけで「記録準備完了」と判断しないでください。
実recorderの必要topicへの`Subscribed to topic`成立を待ってからGUI比較を始めます。
解析区間がbag全体の記録範囲に収まらない場合、coverage.jsonに理由を保存し、
CSVのcoverage_valid=falseおよび終了code 1で無効区間を知らせます。
この区間の低い固定区間Hzは、記録開始前の時間を分母に含んだ可能性があります。
勝手に区間を切り詰めて正常に見せることはせず、元の集計も残します。

## writerの流入帯域と滞留回復

圧縮前byte統計付きの`async_writer.csv`を以下で解析できます（ROS不要）。

```bash
python3 src/pm_evaluation/tools/summarize_writer_backlog.py \
  --csv /path/to/stopped_bag/async_writer.csv
```

受理/保存byteの隣接差分から平均帯域を出し、sample上の最大滞留と終了時の
未保存量を表示します。旧CSVの列不足や累積counterの逆行は拒否します。
1 Hz sampleで見えないpeakは`async_writer.yaml`も併用してください。
elapsed_sはwriter起動からの時計で、bag先頭やheaderの時刻ではありません。
最終pending=0だけでは全試験期間の成功やpublisher側の欠測ゼロを保証しないため、
completion.json、実件数・CRC、独立probeと併せて判断します。

## RAM容量とSSD停滞の計画

`tools/estimate_recording_memory.py`はROSを起動せず、停止済みbagの実測容量と
期間から有限RAMの記録可能時間・SSD停滞中のFIFO容量を概算します。
圧縮後MCAP容量と圧縮前MCAP容量を別々に指定してください。
非同期FIFOは圧縮前CDRを保持するため、圧縮後の小さい帯域では見積もれません。
圧縮前MCAPにはrecord overheadも含まれ、厳密なCDR流入量とは異なります。

```bash
python3 src/pm_evaluation/tools/estimate_recording_memory.py \
  --file-bytes 1545339241 --uncompressed-bytes 2279051065 \
  --duration-s 79.558 --pause-s 12.3116 \
  --buffer-mib 512 --ram-budget-mib 2048 --safety-factor 1.5 \
  --message-count 107390 --message-limit 8192
```

この例の512 MiBは評価用で、recorderの既定値を変更する指示ではありません。
item上限の容量も概算し、byte上限との小さい方を使用します。
平均帯域での容量計画だけではburstや将来の長い停滞を保証できません。
RAM上限とは別にDDS履歴・圧縮workspace・OS page cache・他nodeの使用量もあります。
SSD直接保存＋有限FIFOでは、停止後に滞留が解消し、長時間でも増え続けないことを
writer CSVと整合検査で確認してください。上限超過したbagは成功扱いしません。
RAMだけの保存は短時間試験向けで、SSDへのコピー完了前に電源を失うと消失します。
RAM→SSDの継続spoolでも平均SSD保存能力が流入量を上回らなければ、いずれ満杯に
なります。write完了・metadata確定と電源断に耐える永続化は別の性質です。

停止したbagの件数整合性はpm_bringupのverify_bag_integrityで確認できます。
bag infoだけではmetadataと実message件数の不一致を見逃す場合があります。

probe_process_wait.pyの--threadsを指定するとDDS/workerを含む全threadの待ち先・schedstatも
保存します。mainのfutex待ちだけではwriterの状態を断定できないための追加診断です。
`--threads --syscalls`を必要時だけ指定するとwriterのsyscall番号・引数と
第1引数のfd候補も記録します。診断負荷を抑えるため既定OFFで、DDS threadの
syscallは読取りません。syscall番号は
CPU archに依存し、第1引数がfdではないcallもあるため単独でI/Oと断定しません。
読取り不能は`syscall_error`に記録し、root権限やptrace設定を要求・変更しません。
読み取り頻度・thread数に応じて診断負荷が増えるので、必要な試験だけ指定してください。

## 全topicの空白・OAK診断の確認

複数のsummary.jsonは、ROS環境を必要としない比較CLIで一覧化できます。

```bash
python3 src/pm_evaluation/tools/compare_delivery_audits.py \
  --trial before=/path/to/before/summary.json \
  --trial after=/path/to/after/summary.json --output /path/to/new_comparison.csv
```

全topicの件数、header Hz、最大header/記録間隔と、独立IMUプローブの
`probe_only / probe_count × 100`を出力します。この割合は観測済みmessageの未記録率で、
真のpublish欠測率ではありません。プローブなしは0%ではなく空欄です。
同一負荷・同一期間の反復試験で比較し、開始/停止や診断負荷の差も記録してください。

Foxy環境をsourceしてから、停止・metadata確定済みMCAPを読み取り解析します。

```bash
python3 src/pm_evaluation/tools/audit_bag_delivery.py \
  --bag /path/to/mission_bag --output /path/to/new_audit \
  --profiles src/pm_bringup/config/bag_profiles.yaml
```

summary.jsonに全topicの記録時計・header時計の間隔、OAK sequence診断の先頭/末尾/増分、
profileにあるが未記録のtopic、MCAP各ファイルの記録範囲を保存します。
OAK画像/IMUと対応診断のheaderも共通区間で双方向に照合します。
--probe-csvを指定すると独立IMUプローブとの照合を追加します。
windows.csvは既定5秒窓の記録件数です。
largest_record_gapsは各topicの上位5つの記録空白の位置をbag先頭からの秒で示します。
steady_topicsは全topic共通の記録時計で先頭10秒・末尾2秒を除いた指標です。
--settle-s/--tail-sで変更でき、短すぎるbagはsteady_interval.valid=falseになります。
--end-s 50を指定すると、第二bagの停止時間にかかわらずbag先頭から50秒までに制限できます。
全期間のtopicsも残すので、起動時の欠落を定常指標から除外して隠すことはしません。

画像・点群を排他的分離した場合は、mainに--additional-bag /path/to/bag_bulkを追加し、
全topicを合算して監査できます。metadataが示す全ファイルを読むため、両recorderを
停止・保存検証してから実行してください。同じbagの二重指定やbag間のtopic重複は拒否します。
これは読み取り解析であり、統合再生機能ではありません。
イベントtopicや遅れて起動するtopicは、少数件・先頭windowの0件でも直ちに欠測ではありません。
headerのない型は記録時刻だけを評価します。画像のpayloadを展開せずCDR1の先頭Headerを読む
許可リストを使い、TFMessage等の配列を誤読しません。CDR2はheader評価対象外です。
単独bagにはpublish総数の真値がないので、Hzや長い空白だけで欠損率を確定しません。
sequence gapもdriverの意図的な間引き、driverまでのdrop、診断topic自体の未記録を分けて解釈します。

## 実機を使わない再記録比較

`tools/replay_record_delivery.py` は、既存MCAPからIMU、またはIMU＋画像を再生し、
別のbagへ記録する。入力と出力のtopic・payload SHA256・重複件数を照合する。
元bagに既にある欠測を、新しいrecorderが失ったmessageとは数えない。
messageの記録時刻は再記録で変わるため、時刻の一致ではなく内容で比較する。

Foxy、外部workspace、main workspaceの順にsourceしてから実行する。
開発PCではFoxyコンテナ内、Jetsonではホスト上で使用する。

```bash
python3 src/pm_evaluation/tools/replay_record_delivery.py \
  --bag /path/to/source_mcap_bag --output /path/to/new_result \
  --mode images --storage mcap --duration 40
```

- `--mode imu`: `/oak/imu/data`、`/wit/imu`だけ。
- `--mode images`: 上記とstereo左右、color、depthのraw画像。
- `--storage mcap` / `sqlite3`: recorder保存形式。入力はMCAPのみ。
- `--duration`: 最初の選択messageからのbag秒数。再生速度は1倍。
- `--domain`: 専用ROS domain（既定78）。他のプロセスが使っていない番号を指定する。
- `--qos-file`: recorderのQoS比較用YAML。省略時は自動QoS。
  使用したYAMLは出力ディレクトリへコピーし、実際のrecorderコマンドも保存する。
- `--separate-imu`: `--mode images`時だけ使用可能。**同じ入力を再生したまま**、
  IMUだけを`bag_imu`、画像だけを`bag_images`に記録する2プロセス比較。
  topicは排他的に割り当て、両bagの内容を合算して期待値と照合する。

`summary.json` のexpected/recorded/missing/extraをtopic別に確認する。
`player_wall_s` はplayerの起動・終了、約1秒のdiscovery待ちと最後のpublish後2秒の待ちを含む壁時計時間。
大幅に長い場合は入力を実時間で再生できていないため、実機の帯域負荷と同一視しない。
開始・停止のdiscovery/配送による欠測も比較結果に含まれる。

最後のpublish後2秒はpublisherを維持する。これは最終burstを即座のpublisher破棄で
失わないための比較用対策で、全subscriberのACKを保証するものではない。
比較結果のmissingは再生→DDS配送→記録のどこかで失われたmessageであり、
単独ではstorage内部の欠落と断定できない。

recorderはcacheなし・分割なしにする。QoSは明示指定しない限り自動選択。全条件でplayer終了後3秒待ち、
自分が起動したprocess groupだけをSIGINTで停止する。metadataとbag infoを確認してから
照合する。異常終了時はログを残して例外とし、強制終了によって成功扱いにしない。
既存の出力ディレクトリは上書きしない。

プロセス分離時は各recorderのログ、bag info、metadataを別々に検査する。
`summary.json`の`recorders`に全コマンドと終了コードを残す。既存互換の単数形
`recorder_command`/`recorder_exit`は先頭recorderの値なので、分離条件では
`recorders`全体を確認する。停止中に例外が起きても他方の停止を試み、例外は戻す。

```bash
python3 src/pm_evaluation/tools/replay_record_delivery.py \
  --bag /path/to/source_mcap_bag --output /path/to/new_split_result \
  --mode images --storage mcap --duration 40 --separate-imu
```

これは画像の書き込み待ちとIMU記録が同じrecorderを共有する影響の比較用である。
2プロセスは依然として同じCPU・ディスク・DDS環境を共有するため、完全な資源隔離ではない。
productionのmission記録を自動的に分離する変更ではない。採用には複数bagの関連付け、
開始/終了保証、再生時の統合方法、追加CPU負荷を別途検証する必要がある。

制御topicは選択できず、センサ・EKF・車両のlaunchも起動しない。
localhost限定の専用domainにするが、同じdomainで別の試験を並行実行しない。
replayerはRELIABLE publisherであり、実際のセンサQoS・executor・物理的な揺らぎと
同一ではない。また本試験は点群等の全mission負荷を再現していない。
ここで欠測がないことだけで、実機の問題が解消したとは判定できない。

## 実機での受信・待機診断

通常の性能確認では独立購読プローブを追加しない。まず停止済みbagの既存OAK診断を
`audit_bag_delivery.py`で照合する。診断との一致率`observed_retention_percent`は、
共通header区間の「記録済み診断に存在するmessage」の保存率であり、全発行数に対する
保存率ではない。診断側も欠落し得るため逆方向`source_only`を併記し、
`all_publish_retention_percent`は不明を表すnullにする。先頭・末尾の区間外欠落も含まない。
重複headerを含む件数、連続欠落数、欠落headerの最初と最後の時間差を残す。

`steady_stereo_header_pairs`は左右画像の同一header対応を双方向で示す。
左右が同時に欠落した場合や、VIO自身の購読欠落はこの照合だけでは検出できない。
`steady_camera_info_header_pairs`は画像と同一stampで発行されたCameraInfoの対応を
color/depth別に確認する。RGBとdepth相互の同一stampは要求しない。
`record_gap_evidence`は大きな記録空白の両端についてheader間隔と受信ageの変化を出す。
記録時計はrecorder受信時刻でありSSD書込み完了時刻ではないので、これだけでSSD停滞を
断定しない。大きな記録空白と小さいheader差の組合せは到達の遅れを示すが、途中の
全発行が保存された保証ではない。既存summary項目は互換性のため保持する。

`tools/probe_imu_delivery.py` はBEST_EFFORT独立購読で、受信時計とheaderをCSVへ残す。
`tools/probe_process_wait.py` は指定PIDの主スレッド待機とprocess I/Oを観測する。
いずれもcallback/採取中のディスク書き込みを避け、メモリの上限を設けている。
診断自体の負荷はゼロではなく、A/Bで同じ条件にする。
追加プローブは既存診断で不足するときだけ使用し、プローブなし条件との負荷差を確認する。
影響のある条件を通常運用の性能評価へ流用せず、比較基準がないtopicの保存率は不明とする。

これらはデータ到達・記録欠測の切り分け用であり、IMU校正やVIO品質の判定ではない。
DDSのworker待機や全スレッドの詳細を主スレッドwchanだけから断定しない。

## 実機で同時記録した2つのbagの比較

`tools/compare_live_imu_bags.py` は、同じpublisherから同時に記録したmission bagと
IMU専用MCAPを比較する。入力bagは変更せず、ROSノードも起動しない。
必ず両recorderを停止し、metadataとbag infoを確認してから使用する。

```bash
python3 src/pm_evaluation/tools/compare_live_imu_bags.py \
  --mission-bag /path/to/mission_bag --imu-bag /path/to/imu_bag \
  --probe-csv /path/to/delivery.csv --output /path/to/new_comparison.json
```

topicごとの共通header区間で、header整数nsとpayload SHA256を件数付きで照合する。
`only_sidecar`はIMU専用bagだけ、`only_mission`はmissionだけにあるmessage数。
一方を真値にせず両方向を残す。開始・終了時刻の違いによる区間外のmessageは除くため、
この比較だけではbag先頭・末尾の欠測まで検出できない。

`--start-header-s`/`--end-header-s`でepoch秒の定常区間も指定できる。
同じ時間窓を両bagへ適用し、実行開始後20～60秒等で初期化・停止の影響を分ける。
範囲が重ならない場合やtopicがない場合は、成功とせずstatusに残す。

probe CSVを指定すると、`probe_headers_not_mission`/`probe_headers_not_sidecar`で、
独立購読にはあるが各bagにはないheaderも数える。probe自体にも欠測があり得るため、
逆方向の件数も保存する。probeはheaderだけの照合で、payload一致の証明ではない。

周波数・最大header間隔と、最大記録時刻間隔・headerからの記録ageは分けて出力する。
OAKのheader時計とホスト時計にオフセットがあればageは負にもなり、負値を「遅延なし」
と決めつけない。header順への整列で隠れないよう、記録順のheader非増加も計数する。
既存の出力JSONは上書きしない。

## VIO callback停滞とbagの受信間隔の照合

OpenVINSの`imu_input_diagnostics:=true`で得たcore logを、停止済みmission bagと
同じIMU headerで照合する。独立購読を増やさず、記録完了後にだけ実行する。

```bash
python3 src/pm_evaluation/tools/analyze_vio_callback_stalls.py \
  --bag /path/to/bag --core-log /path/to/core.log --output /path/to/new_analysis
```

`callback_delays.csv`には、headerが連続していた40 ms超のcallback開始間隔と、
同じ2サンプルのbag受信間隔を並べる。nativeのdouble時刻と小数3桁のdeltaの丸めを
考慮し、headerは2 us以内だけ対応付ける。対応できなければ空欄であり、欠落の断定ではない。
recorder受信時刻はpublish時刻でもSSD書込み完了時刻でもないので、間隔が近い／異なる
ことだけでDDS・OS・driverのどこが原因か決めない。
既存の`/oak/diagnostics/imu_packet`に`host_receive_time_ns`があれば、同じ2サンプルの
driver側の時刻差も`driver_post_publish_gap_ms`に出す。現在のPython driverでは
IMU publish後の診断組立時に時計を読む値であり、名称に反してUSB受信時刻そのものではない。
driver側も間隔が空くならVIO callback単体だけでなく、driverの処理・publish待ち・OS待ちを
優先して調べる。項目がなければ空欄であり、新しい診断topicは追加しない。
`summary.json`の`largest_record_gap_evidence`にはbag受信の大きい空白も同じheaderで
照合して残す。これにより、VIO callbackと上流が共通に止まるケースと、bag受信だけが
遅れるケースを分けて検討できる。重複headerは最初の受信に固定する。
既存解析は上書きせず、新しいoutputディレクトリを指定する。
`input_gaps.csv`は`IMU_INPUT_GAP`の両端と、その間のbag/driver診断の件数・時刻差を
残す。内部サンプル数は両端を除いた固有header数で、欠測区間のIMUがdriver側で
通常周期で出たか、burstで出たかを確認する。対応が不明な場合は空欄を残す。
安全停止の理由・推定器のbuffer削除はこの表だけで判断せず原文も確認する。
画像callback内部、message_filters内部、deserialize前の待ちは別々に検討する。
