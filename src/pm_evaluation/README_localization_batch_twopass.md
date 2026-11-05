# 自己位置推定一括解析・２回再生版

既存の`analyze_localization_batch`をコピーして作成した別版です。既存コマンドの
動作は変更しません。開発PCのFoxyコンテナで実行し、実機node・Jetsonには使いません。

## 処理と再生回数

1. **PASS 1**: 元bagの左右画像・OAK IMUを１回再生し、独立したOpenVINSの
   `online-cam-calib_on/off`へ同じ入力を配信します。全条件の購読成立後に開始し、
   namespace・出力bagを分離します。変更する校正flagは従来どおり`calib_cam_extrinsics`だけです。
2. **hub合成**: wheel・Wit・GNSS・固定TFを１組と、条件別のVOを、記録ns順に合成します。
   ファイルの読出し/保存だけで、等倍再生待ちはありません。CDR・headerは変更しません。
3. **PASS 2**: hubを１回再生し、全VO条件・磁気条件・５方式・当時設定・wheel比較の
   EKFを同時実行します。動的TF・サービス・出力はgraphごと、VO入力は校正条件ごとに
   分離し、固定TF・wheel・Wit・GNSSを共有します。
   DDS discoveryが肥大化しないよう既定では5 graphずつdomainを分け、1つのbag iteratorから
   各domainのContextへ同じ記録時刻で配信します。domain数が増えても再生走査は1回です。
4. 数値bagを方式別の元topic名へ分離し、従来と同じ意味のCSV・XY地図図・z図・状態図を保存します。
   GNSS品質記号、始終点・方向、欠落/異常注記も維持します。

全計算が未完了の場合も、１試行につき等倍再生は最大２回です。`--wheel-per-mode`を
付けても増えません。記録rawだけを使う場合はPASS 1不要で１回、全cache hitなら０回です。
読出し・hash照合・hub合成・描画のためのファイル走査は別途あります。

既定の全条件では26個のEKF graph（各graph内に複数node）を同時に走らせます。
既定では6つのdomain（192〜197）を使います。`--graphs-per-domain 1..5`、`--domain`で
調整できます。使用範囲が232を超える場合は起動前に拒否します。実機のdomainとは分離してください。
小行列を扱うPython nodeのBLASだけを1 threadに固定し、起動時の数千thread生成を避けます。
OpenVINSやEKFの推定parameterを変更する設定ではありません。
`--max-ekf-graphs 64`が上限で、超える場合は再生を分割せずエラーにします。
既定値は資源上限の保証ではありません。CPU/RAM余裕と数値結果を短区間で確認してください。

## 実行例

コンテナでビルドした後、実行用shellでoverlayをsourceします。

```bash
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
cd /workspaces/patasmonkey_ws
colcon build --packages-select pm_evaluation --symlink-install
source install/setup.bash

# まず全件の条件計画だけ確認する（ROS nodeは起動しない）。
ros2 run pm_evaluation analyze_localization_batch_twopass \
  bags/rosbag2_2026_10_30-homearound --plan-only

# １試行45秒で確認する。指定を省いた条件は通常の全条件。
ros2 run pm_evaluation analyze_localization_batch_twopass \
  bags/rosbag2_2026_10_30-homearound \
  --bag rosbag2_2026_10_30-15_09_03 --duration 45

# ７試行を全期間実行する。VOを外したEKFも５方式別に作成。
ros2 run pm_evaluation analyze_localization_batch_twopass \
  bags/rosbag2_2026_10_30-homearound --wheel-per-mode
```

`--wheel-per-mode`を省くと、wheel側は従来どおりseparated_offroadを共通基準とします。
`--duration`・`--rate`・`--vio-variants`・`--magnetic`・`--modes`・`--stages`・
`--initial-yaw-deg`・`--vio-config`等で条件を選べます。既存版の`--ekf-workers`と
`--no-lowband-cache`はありません。全条件の共有を分割しないためです。

## 保存先と再開

既定の結果rootは`src/pm_evaluation/results/<入力root名>_batch_twopass/`です。
既存版の結果rootとは分離します。解析directoryはJSTの`YYYYMMDD_HHMM`です。
同分の衝突は連番、条件識別は`analysis_identity.json`で行います。

- `pass1_vo/`: VO共有再生のjob、配信件数、資源記録。
- `vio_replay/<校正条件>/<解析日時>/`: native出力、初期設定、診断、状態CSV。
- `cache/input_hub/`: 校正条件別rawを含む共通低帯域入力。
- `pass2_ekf/`: 全条件共有の接続計画、nodeログ、配信件数、資源記録。
- `comparison/`、`historical/`、`sources/shared_wheel/`: 方式別bag、CSV、図。
- `summary.json`: 成功/失敗、実行したPASS数、独立graph数、結果への相対パス。

同じ条件で完了済みの数値結果は再利用します。失敗・未完了なら新しい日時で再実行し、
以前の失敗結果は残します。`--no-resume`は新規計算を要求します。新版と既存版の数値cacheは
backendが違うため混用しません。再生したgraphの組合せによる非決定性もあり、bit一致は保証しません。

## 失敗と診断

VO条件ごとの失敗を残し、成功条件だけを後段のVO入力とします。VOが両方失敗しても
`sources`を選択していればwheel＋IMUの計算を続行し、VOのない重畳図へ理由を注記します。
失敗条件があればCLI全体は非0で終了します。速度warningも成功と区別します。

`resources.json`は追加ROS購読を作らず、５秒ごとの/procによるprocess treeのRSS・
CPU累積秒・空きメモリ・loadを記録します。`delivery.json`は配信件数と最大スケジュール遅延です。
publish件数だけでは全nodeへの受信を保証しないため、数値結果・adapter出力・診断も確認します。
資源不足なら条件を絞るか、再生回数を増やさず`--rate 0.5`等で再生を遅くできます。
倍率を変える場合も結果の比較検証が必要です。間引き・高速再生・parameterのrate変更は行いません。第2段が実時間scheduleへ
追い付かない場合は、追い付きburstを避けて実時間だけを延長し、延長秒数を記録します。
bag記録時刻・header・/clockに与える時刻値は変更しません。

## 保存容量の既定方針

ROS recorderが保存するMCAPは既定でZSTDの可逆chunk圧縮（CRC保持）を使います。
Foxyの既存overlayが提供する`PM_MCAP_PRESET_PROFILE=zstd_fast_crc`をrecorder childだけへ
指定します。transportのfile圧縮ではないので拡張子は`.mcap`、既存readerで読めます。
Pythonで作るVO・入力hub・分離後bagは従来からZSTD圧縮されており、そのまま使います。

共有中間bagは、全ての方式別bagを再読し、改名後のtopic・型・記録ns・CDRバイト列・
メッセージ順序をSHA256と件数で照合した後に削除します。分離照合に失敗した場合は
共有bagを残します。診断ログ・`group.json`・`delivery.json`・`split_verification.json`・
`shared_storage.json`は残します。最終数値bagがあるため、共有bag削除後もcache再利用できます。

```bash
# 調査時だけ共有bagを保持する。通常は付けない。
--keep-shared-bags

# recorder圧縮を無効にする比較用。Python生成bagのZSTDは維持される。
--mcap-compression none
```

圧縮overlayのcapability markerがない場合は、黙って非圧縮記録せず明示的に失敗します。
`/workspaces/ros2_ws/install/setup.bash`をsourceしてください。既存の結果は変更・圧縮・
削除しません。これらの既定値は、今後新たに記録/分離するbagへ適用します。

保存オプションは新しい記録にだけ作用し、cache hitした結果を再圧縮・再記録しません。
削除済み共有bagを作り直して保持する場合や圧縮条件を比較する場合は、
`--keep-shared-bags --no-resume`、`--mcap-compression none --no-resume`等を指定します。

## 初期方位の取得

両版共通の`localization_heading.py`を使います。記録済み`/localization/initialization/status`の
READYをbag開始付近で確認し、先頭`/odometry/local`のyawを直接初期値へ与えます。
このlocalは起動時に地球方位へ合わせられているため、`map→odom`の回転は加えません。
このTFにはglobal EKFの方位補正が入り得るため、初期方位の根拠にはしません。
起動後に車体が旋回していることも考慮し、READY内の起動時seedを全bagへ固定適用はしません。
READY不在/未完了/古い通知や不正quaternionの場合は明示指定を求めます。
磁気補正ONはこのlocal yawを使用し、OFFだけ保存offset_deg分を除く既存の比較を維持します。

初期状態が変わるとEKF cache keyも変わり、誤初期方位の結果は再利用されません。
VOの画像/IMU再実行は初期yawに依存しないので、同条件の正常なVO cacheは再利用できます。
その場合は`--no-resume`を外します。既存プロセスは起動時のコードを使うため、
修正は次回起動から反映されます。既存結果を改名・削除・回転上書きはしません。


### 磁気方位補正の検証

初期yawはENU初期化READYを確認した先頭local姿勢を使用し、map→odomの
回転を重ねない。mag_onは補正済みyawを維持し、mag_offは保存offset_degを
ラジアンへ変換して加える。実機のyaw = π/2 − (bearing + offset)から
offsetを取り除くため、この符号となる。両解析CLIはlocalization_heading.pyの
共通処理を使う。非有限のyaw/offsetはEKFを起動する前にエラーとする。
旧magnetic_offset_zero_approxは描画の回転近似であり再計算結果ではない。
