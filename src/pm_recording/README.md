# 単一bagの非同期writer（ROS 2 Foxy）

`pm_recording/async_recorder`は標準Foxyの`Rosbag2Transport`を再利用し、
writerだけを有界FIFO＋専用保存threadで包む。画像のstorage write中にも、
transportの受信callbackが次のIMU／EKF messageを取り込めるようにする。
topic探索、型、CDR、受信時刻、購読QoS、MCAP、分割は標準経路のまま。
topic登録／削除とmessageも同じFIFOで処理し、storageの競合を避ける。

```bash
# センサcoreが別途起動済みの状態で記録のみ開始する。
ros2 launch pm_bringup mission_bag.launch.py \
  writer_backend:=async bulk_sidecar:=false
```

通常writerへの切替は`writer_backend:=standard`。
以前の分割方式へ戻す場合は`writer_backend:=standard bulk_sidecar:=true`とする。
単一bag方式の検証では`bulk_sidecar:=false`とし、二つの方式を併用しない。
本packageはセンサ・モータ・車両指令を起動しない。

## 容量・停止・失敗の意味

- FIFOは既定64 MiBかつ8192 item。byte上限はCDR payload合計で、書込み中1件を含む。
  ROS/DDS、object overhead、MCAP内部bufferは別にRAMを使う。
- 保存先は1つのbag directory。通常の2 GB（2000000000 byte）分割による複数MCAPはあり得る。
- 既定容量は初期比較用の有限予算。画像の約1秒規模のburstを想定するが、
  messageサイズと入力帯域次第で保持時間は変わる。実測peak_bytesを見て評価する。
  GUIあり試験では64 MiB、128 MiBで容量超過を観測し、192 MiBでもitem上限に
  達した例がある。この方式は検証用であり、キュー増量による解決を保証しない。
- FIFOを満杯にして無限に受信を待たせたり、古いmessageを黙って捨てたりしない。
  超過で録画を異常停止し、`async_writer.yaml`に失敗を残し、非zero終了する。
- SIGINTでは購読停止後に受理済みmessageを全flushしてmetadataをcloseする。
  `accepted == written`だけでDDS以前の欠測がないとは言えない。
- parentの`record_trial`は従来どおりmetadata・bag info・実件数も照合する。
- `async_writer.yaml`とCSVの`accepted_bytes` / `written_bytes`は圧縮前CDRの
  累積byte。CSVの隣接行の差分を経過秒で割ると、受信帯域と保存帯域を比較できる。
  差が増え続けるか、停滞後に回復するかで有限FIFOの追従性を判断する。
  旧バージョンのCSVにはこの列がないため、ゼロ流入と解釈しない。
- Foxy標準`max-cache-size`は0のまま。MCAPの非zero cacheで観測した保存欠落を
  回避し、別機構のbyte有界FIFOを用いる。`/opt/ros/foxy`は改変しない。

## 可逆圧縮の比較（任意）

### SSDとRAMの併用について

`writer_backend:=async`でSSD上のbagを指定する方式は、既に「有限RAM FIFO＋
SSDへの継続保存」です。bag全体をRAMへ置く方式とは異なり、正常に追従すれば
記録時間に比例してRAMが増えません。byte上限だけでなく8192 itemの上限も
あるため、小さなmessageが大量に滞留する場合はbyte予算より先に失敗します。
停止時のflush待ちと、途中の最大滞留量・停止後の件数整合を必ず確認します。
`writer_queue_bytes`と`writer_queue_items`で両上限を試験条件として指定できます。
既定は64 MiB/8192 itemで変更していません。512 MiB/8192 itemの長めの試験は
約166 MiBの滞留でitem上限に達しました。byte上限だけの増量では不十分です。
item上限の増量もmessage objectの追加RAMを使い、SSD能力不足は解決しません。

2026年10月のGUIあり試験で圧縮後約19.4 MB/s、圧縮前MCAP約28.6 MB/sを
観測しました。最大約12.3秒の停止を平均帯域×安全係数1.5で見積もると約505 MiBです。
512 MiBはこの停滞に対する比較用容量で、既定値への採用や無欠損保証ではありません。
`pm_evaluation/tools/estimate_recording_memory.py`で別の入力条件も概算できます。
同じ条件でSSD停滞が短くなった場合、成功を容量増加だけの効果と断定できません。

bag全体を`/dev/shm`へ保存して後でコピーする方式は短時間の切り分け向けです。
圧縮後でも記録時間に比例してRAMを使い、再起動・電源断で未退避データが消えます。
連続spool方式を追加するなら、RAM容量上限、SSD退避の追従、コピー検証、
metadataの参照先、停止・異常時の復旧を一緒に実装する必要があります。
平均SSD帯域が流入量以下の場合は、spoolでも有限RAMを使い切るため解決しません。

外部workspaceの`rosbag2_storage_mcap` overlayが対応している場合のみ、
`mcap_preset_profile:=zstd_fast_crc`でZstd Fastest＋chunk CRCを指定できる。
未対応overlayなら起動前に拒否する。既定は`default`で従来の非圧縮保存。
設定はrecorder childへだけ渡し、IMU・画像・DDSの設定は変えない。
圧縮で記録量が減ってもfilesystemの長い待ちを必ず防げるわけではない。
実件数・CRC・復元内容と、CPU増加、FIFO容量超過の有無を合わせて評価する。

## 限界

transportのgeneric subscription／executorとDDSは変更しないため、
それらが原因の遅延やpublisher以前の欠測を必ず改善するものではない。
bag記録時刻は標準transportのcallback受信時計であり、storage flush時計ではない。
記録間隔短縮だけでディスク遅延ゼロと判断せず、`max_write_seconds`も参照する。
未知のQoS項目は無視せず拒否する。現在対応するYAML項目はhistory/depth/
reliability/durability。起動時に未発行topicは通常のdiscoveryで後から購読する。

## ビルド・テスト

### 同期writeの遅延診断（試験用）

`timing_recorder`は標準の同期writerを維持して診断する。非同期FIFOは使わない。
通常launchの既定recorderを置き換えるものではない。

```bash
ros2 run pm_recording timing_recorder -o /tmp/timing_trial \
  --trace on --qos-profile-overrides-path /path/to/bag_qos.yaml \
  /oak/imu/data /oak/stereo/left/image_raw
```

`--trace off`との交互比較で観測負荷を評価する。両条件ともQoSは指定YAMLのまま。
停止後の`write_timing.csv`はOAK IMU全件と10 ms以上のwriteを最大30000行保存する。
上限超過件数は`write_timing_info.txt`の`omitted`へ残す。CSV保存は録画終了後のみ。
`record_ns`はtransportが採番したbag記録時刻、`write_enter_ns`は同期write入口の
system clock [ns]、`write_duration_ns`はsteady clockで測った下流write所要時間 [ns]。
入口までの遅延にはWriterの排他待ち等が含まれ、DDS到着時刻は測っていない。
入口以前の原因をDDS・executor・callbackへ個別分解できたとは解釈しない。
共通CLIのqueue指定は診断targetでは使用せず、標準cacheも0を維持する。
共通の`async_writer.yaml`/CSVはこのtargetでは同期write受理/完了件数として読む。
`verified`だけではDDS以前の欠測や全topic購読成立を保証できないため、別途bag監査が必要。

### 受信取り込み境界の診断（試験用）

`librcl_receive_trace.so`は明示的な`LD_PRELOAD`でrecorderだけへ適用する。
通常launch・QoS・画像サイズ・画像周波数・executor構造は変更しない。
`rcl_wait`と`rcl_take_serialized_message`の元関数をそのまま呼び、最大300000件を
固定メモリへ保存する。CSVのファイルI/Oはprocess正常終了時のみ行う。

```bash
LD_PRELOAD=/path/to/install/pm_recording/lib/librcl_receive_trace.so \
PM_RCL_TRACE_PATH=/tmp/rcl_receive.csv \
ros2 run pm_recording timing_recorder -o /tmp/receive_trial --trace on \
  --qos-profile-overrides-path /path/to/bag_qos.yaml /oak/imu/data /oak/color/image_raw
```

`kind=1`はwait、`kind=2`はtake。`topic=1`はOAK IMU、`topic=2`はcolor、
`topic=0`はその他。`begin_ns/end_ns`はsteady clock、`wall_ns`は終了後の
system clock [ns]。`ready`はwait成功時にNULLでなかったsubscription数、
`imu_ready`はその中にIMUがあったかを示す。DDS到着時刻を直接示すものではない。
take成功時のみ`bytes/source_ns/received_ns`を保存する。RMWが時刻を提供しない
場合は0であり、受信遅延の計算には使用しない。戻り値`result`は元のrcl_ret_t。
CSV冒頭に省略件数を残す。SIGKILLや異常終了ではCSVが保存されない場合がある。
他のprocessへグローバルにpreloadせず、診断なしとの交互比較で観測負荷を確認する。

`PM_DDS_TRACE=1`を追加すると、FastDDS 2.1.4のRELIABLE readerのDATA入口
(`kind=3`)、DATA_FRAG入口(`kind=4`)、HEARTBEAT入口(`kind=5`)も記録する。
既存CSV列は保持し、`gid_hi/gid_lo`（GUIDの16 byteを8 byteずつ数値化）、
`sequence/sequence_last/fragment`を末尾に追加する。take側のGIDと照合し、
topicを特定する。DATAではsequence、HEARTBEATでは最初/最後のsequenceを保存する。
IMUとcolorの最初のtake時に、rmw_fastrtpsの購読から実reader GUIDを取得する。
以後はその2つのreaderと対応するpublisherのGUID組だけを計測する。
対象外reader/writerへのDATA配送も存在するため、
全readerを計測すると診断負荷や履歴量が過大になる。購読識別前のDDS入力は
計測しないので、開始直後の全DDS入力を数える用途には使用しない。
1つの記録区間では同じpublisherが継続する前提で、最初のGUIDを保持する。
camera nodeを再起動してpublisher GUIDが変わる試験では診断recorderも起動し直す。
DATA_FRAGのbytesは全体sampleSize、fragmentは先頭fragment番号。
reader入口でsource時計・GUIDをコピーし、元処理で回収され得るchangeを戻り後に
参照しない。DDS行のwall_nsは入口のsystem clockで、rcl行は出口のsystem clock。
この入口はDDS readerへの到着であり、USB入力や最初のnetwork packet到着ではない。
全履歴の固定領域は約36 MBで、録画中に追加ファイルI/Oや購読は行わない。

診断hookの対象symbolはFoxyのFastDDS 2.1.4の公開header/exportと照合したもの。
別バージョンへ無条件にpreloadせず、ビルドとsymbolの一致を確認する。
共有メモリ経路の欠測・history競合・再送をログだけで断定せず、入口のsequence
逆転/重複、受信処理時間、rcl takeまでの間隔を合わせて解釈する。

Foxyコンテナ内で外部workspaceをsource後に本packageとpm_bringupをbuildする。
性能比較用のrecorderは`--cmake-args -DCMAKE_BUILD_TYPE=Release`を指定する。
MCAP pluginの最適化条件も記録し、最適化なしのwriterとの比較を混同しない。
`colcon test --packages-select pm_recording`は遅い偽物storageを用い、
FIFO順序、停止flush、容量超過、storage例外、cache禁止を検査する。
