# 自己位置推定bagの一括解析

`analyze_localization_batch`はmetadata.yaml単位で再帰探索し、分割MCAP/sqlite3を一試行として扱います。
入力の画像・点群は読出し対象にせず、推定に必要な低帯域topicだけを再生します。
**Foxyコンテナで実行**してください。実機・車両制御・ジョイスティック・センサドライバは起動しません。
別ROS domainとROS_LOCALHOST_ONLY=1を使用します。同時に別の解析を行う場合はdomainを分けます。

## 実行例

コンテナ内で通常のoverlayをsourceし、workspaceへ移動して実行します。

```bash
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
source /workspaces/patasmonkey_ws/install/setup.bash
cd /workspaces/patasmonkey_ws

# 全試行、記録図・当時parameter・５方式・ソース比較、磁気補正あり/なし、OpenVINS外部校正ON/OFF。
ros2 run pm_evaluation analyze_localization_batch \
  bags/rosbag2_2026_10_30-homearound

# 実行前に探索対象・入力・全jobだけを確認（ROS推定ノードは起動しない）。
ros2 run pm_evaluation analyze_localization_batch \
  bags/rosbag2_2026_10_30-homearound --plan-only

# １試行・分離方式だけ、先頭30秒で実行確認。
ros2 run pm_evaluation analyze_localization_batch \
  bags/rosbag2_2026_10_30-homearound \
  --bag rosbag2_2026_10_30-15_12_04 --stages recorded comparison sources \
  --modes separated_offroad --duration 30 --vio-variants recorded

# 地図を既存キャッシュから取得。範囲外のキャッシュは拒否する。
ros2 run pm_evaluation analyze_localization_batch \
  bags/rosbag2_2026_10_30-homearound --basemap /path/to/basemap.npz

# 同じコマンドで再開。入力・設定・実装が同じ完了jobは再計算しない。
# --no-resumeで強制再実行。不完全な出力bagは削除せず退避する。
```

ホストからは、上のsource・cd・実行コマンドを
`docker exec -u 1000:1000 patasmonkey_foxy_dev bash -lc '…'`で実行できます。
`--output-root`で保存先を変更できます。既定は
`src/pm_evaluation/results/<入力ディレクトリ名>_batch/`です。

## 出力構成

```text
<解析root>/README.md, summary.json
  <入力rootからの相対bagパス>/
    gnss/                  # RTK状態・起終点・移動方向、GNSS CSV
    recorded/              # 記録されたlocal/global（変更しない）
    historical/<VO条件>/mag_on|off/<当時方式>/<解析日時>/
    comparison/<VO条件>/mag_on|off/<５方式>/<解析日時>/
    sources/shared_wheel/mag_on|off/<wheel方式>/<解析日時>/
      output_bag/          # 実ノードの出力、ゲート、内部加速度、診断
      source_comparison/<VO条件>/   # 新adapterのVOとwheel＋IMU（sources配下）
      xy_full.png, xy_gnss_area.png, z.png, states.png
      *.csv, metrics.json, input_topics.json, job.json, *.yaml, *.log
```

GNSS地図だけは元bag直下へ`gnss_trajectory_osm.png`としても保存します。
このPNGは同名を更新します。元bagのmetadata・分割ファイルは変更しません。
解析rootを入力root配下に置いた場合でも、その中の派生bagは探索から除外します。
推定は等倍・記録時刻順が既定で、headerは変更しません。
`--duration`は最初の選択メッセージ記録時刻からのbag秒です。
初期化やGNSS変換前に打ち切ると図が作れず、jobは失敗として保存されます。

## 方式と比較条件

- `legacy`：従来local設定。
- `separated_offroad`：2D水平EKF＋Wit傾斜＋VO高さ。
- `separated_projected`：車輪・VO速度を水平投影。
- `ekf_3d_no_accel`：3D EKF、加速度直接観測なし。
- `ekf_3d_accel`：3D EKF、重力補償を有効にした加速度も直接観測。

５方式はlocalの比較です。global水平部は車輪・Wit・ゲート済みVIO・GNSSを
個別融合する共通構成、z/roll/pitchは各localから合成します。
そのためglobalの水平軌跡が方式間で同じでも異常ではありません。
`--wheel-per-mode`を付けると、VOを外す比較も各方式で行います。
既定のwheel比較はseparated_offroadの１本です。

全再計算で**新版adapterを実行**します。raw `/ov_msckf/odomimu`が必要です。
旧`/vio/odometry`から不完全な変換・共分散を逆算して代用しません。
`/visual/odometry`など別名topicは一覧へ記録しますが、内容不明のままraw扱いしません。
VOは新adapterで取付変換と初回local整列を受け、描画側で二重に軸変換しません。
既定のVO条件は`online-cam-calib_on online-cam-calib_off`です。左右画像とOAK IMUからOpenVINSを再実行し、
そのraw出力を**全てのVIO依存local/global計算へ供給**します。
`--vio-variants recorded`を明示すれば、記録rawを新版adapterへ通す軽い検証もできます。
記録メッセージそのものの`recorded/`描画は１回だけで、校正ON/OFFには分けません。

再実行はbagのconfiguration_indexからestimator設定と初期校正チェーンを複製し、
`calib_cam_extrinsics`だけをtrue/falseへ変更します。camera–IMU取付回転・位置の
オンライン校正が対象で、内参・時差・IMU内参など他の校正flagは保持します。
使用flagを`vio_replay/<条件>/<解析日時>/config/conditions.json`へ保存します。
設定が見つからない場合は`--vio-config /path/to/estimator_config.yaml`で明示します。
元の設定・外部workspaceは変更しません。再実行のraw bagは再利用するので、
磁気補正や５方式ごとに画像処理を繰り返しません。

OpenVINSの出力受信時に最後に配信済みの入力bag時刻でrawを記録し、
元の低帯域入力と記録順に合成します。wall clockとsensor clockを混在させません。
画像本体は派生入力へ重複保存しません。画像の再生順は元の記録順でheaderは保持します。
過去と同じ内部状態・特徴追跡・スレッド順を完全再現するものではありません。
bag先頭に初期化に必要な静止/励起がない場合はraw出力が得られず、理由付きで失敗します。
その際も元の記録プロットは残し、記録VOでの計算へ黙って置き換えません。

## 磁気補正・疑似初期化

`mag_on`は記録済み初期化statusがbag開始付近（2秒以内）にREADYであることを確認し、
bag先頭のlocal姿勢からyawを直接使用します。localは起動時にENU方位へ初期化されているため、
global補正を含むmap→odomの回転は加えません。
`mag_off`はそこから保存offset_degだけを除去したENU yawを初期値へ与えます。
`yaw_ENU=pi/2-(bearing+offset)`なので、offのyaw変化はoffset_degそのものです。
磁気ベクトルのbias/scale校正は両方で維持します。
記録図は実測された実行結果なので、補正なしの別図を捏造しません。

保存offsetなしの場合は現行校正値で代用し、summaryへ明記します。
READYや有効なlocal姿勢を確認できない場合は、TFから推測せず`--initial-yaw-deg`を明示してください。
READY内のseed_yaw_radは起動時の向きの記録であり、車体が旋回した後のbag先頭yawの代用にはしません。
bag開始時点のyawをinitial_stateへ１回設定する**疑似初期化**であり、
静止判定・磁気平均・初期化サービスを過去に遡って再実行するものではありません。

navsatはこのENU yawを持つglobal odometryを使い自動初期化します。
品質ゲートは製品nodeを使用しますが、実機のdatumサービス・検証完了待ちは再現しません。
初回GPS出力と同時刻fixの差を固定offsetとし、localは同時刻のmap→odomを固定して描画します。
各時刻のglobal補正をlocalへ混入させず、軌跡fit・縮尺調整はしません。

## 当時parameterの再現の限界

`provenance/effective_parameters.json`のsnapshotを優先し、現行ノードへ適用します。
未保存の設定は現行YAMLを使い、代用一覧をjob.jsonへ記録します。
当時設定がなければhistoricalはunavailableとし、成功扱いしません。
**当時の実行コード、bag以前のEKF状態・完全共分散は復元できません。**
加えてadapterは指定どおり新版です。このためhistoricalは「保存parameterを使った
現行実装上の再現試験」であり、当時バイナリの完全再現ではありません。
使用YAML・入力一覧・初期化方法・代用・実装を含むfingerprintを保存します。

## 診断・失敗・再開

- 入力headerの最大空白、最終時刻、件数を保存。
- ゲート入力/出力件数、adapter状態、EKF内部加速度とvy/分散を保存。
- 診断メッセージ数は通過件数や通過率と混同しません。
- GNSSとの差は共通区間のみで、長い欠測を越えて補間しません。GNSSは真値ではありません。
- zは生値と各系列の初期高さからの変化量を分けます。VO相対高さとGNSS高度のdatumは異なります。
- GNSSなし・不足topic・地図取得失敗を理由付きで記録。地図失敗時はNO BASEMAPを図に明記。
- 子プロセスの終了を監視し、SIGINTで順に終了。recorder停止後にmetadataとbag infoを確認。
- 完了jobだけを再利用します。fingerprintはbagのmetadata・サイズ/mtimeと設定・コードに基づきます。
  内容hashを取らないため、bagの内容を変更しmtimeも戻す操作は検出できません。

全試行×５方式×磁気２条件に履歴・ソース比較も加わるため、等倍では長時間かかります。
まず`--plan-only`、次に１試行を短時間で確認してから全期間を実行してください。

### 校正ON/OFFの再計算だけを絞って試す

```bash
ros2 run pm_evaluation analyze_localization_batch \
  bags/rosbag2_2026_10_30-homearound \
  --bag rosbag2_2026_10_30-15_09_03 \
  --stages comparison --modes separated_offroad \
  --vio-variants online-cam-calib_on online-cam-calib_off --magnetic on
```

全条件では２つのOpenVINS再実行に加え、磁気２条件×方式・履歴・ソース比較が走ります。
OpenVINS rawの欠落・停止と再初期化失敗は、その後のEKFでも同じ条件として反映されます。

## 2D共分散境界と数値品質検査

FoxyのforceTwoDが平面外分散だけを微小値へ変更すると、元の3D交差項から
不定な観測共分散が生じ得ます。2Dのlocal/globalへは専用nodeで(vx,vy,wz)の
周辺共分散を取り出して渡します。3DのVIO出力そのものは変更しません。
投影方式では水平投影を行った後でこの処理を実施します。

各jobのquality.jsonで、出力共分散の負分散・非半正定値・非有限速度を検査します。
不正なら図は証拠として残しますが完了印を付けず失敗とします。3 m/s超は物理的な
ドリフトの疑いとしてwarningとし、数値不正とは区別します。

## １回の再生で５方式を並行実行する

方式比較は既定で`--ekf-workers 5`です。同じVO条件・磁気条件の５方式を
１回の低帯域bag再生で走らせます。`/clock`と元センサ入力・固定TFは共有し、
各方式のlocal/global・adapter・ゲート・動的TF・set_poseサービスは分離します。
内部では`/evaluation/case_N/`ごとのROS namespaceと明示的なtopic remapを使います。
frame名自体はodom/base_linkの意味を保ち、異なる方式の動的TFが混ざらないようにします。

```bash
# ５方式を共有再生（既定）。まず短い区間で動作確認する例。
ros2 run pm_evaluation analyze_localization_batch \
  bags/rosbag2_2026_10_30-homearound \
  --bag rosbag2_2026_10_30-15_12_04 --stages comparison \
  --vio-variants recorded --magnetic on --duration 35 --ekf-workers 5

# 従来の逐次実行へ戻す。同時実行数は1〜5から選べる。
ros2 run pm_evaluation analyze_localization_batch \
  bags/rosbag2_2026_10_30-homearound \
  --bag rosbag2_2026_10_30-15_12_04 --stages comparison \
  --vio-variants recorded --magnetic on --duration 35 --ekf-workers 1
```

共有再生の接続計画・起動ログ・一括記録bag・配信件数・時間は、各試行の
`shared_replays/<VO条件>/<磁気条件>/<解析日時>/group_.../`に保存します。
方式別の出力bagは元のtopic名へ戻して保存し、既存のCSV・描画処理を使います。
各方式の`execution.json`から共有groupへ辿れます。
共有記録は方式別bagへ分離し、全recordの時刻・型・CDR・順序を照合した後に削除します。
`--keep-shared-bags`を指定した場合は両方を残すため容量が増えます。

完了済み方式は再利用し、未完了の方式だけで共有groupを作ります。
入力bag・topic選択・再生倍率・durationが異なるjobは同じgroupに混ぜません。
共有groupのnodeが異常終了した場合は、そのgroupの未完了方式を失敗として残します。
他方式の結果を流用して成功扱いにはしません。

現段階の共有対象は`comparison`です。`historical`と`sources`は逐次実行を維持し、
OpenVINS ON/OFFも各１回ずつ順次再実行します。５方式部分の再生回数は5→1へ減りますが、
解析全体が一律５倍速になるわけではありません。既定の再生待ち部分の概算は30T→14Tです。
実行結果の一致は共通時刻で比較し、ROS timer・callback順の差による非決定性も検査します。

共有再生はgraph登録だけで開始せず、入力publisherが必要数のsubscriberへ接続したことも
確認します。配信rawがあるのにadapter出力が0件なら、TFまたは初期整列の不良として
jobを失敗にします。短い初期整列待ちやROS timerの実行順による数件の出力差は残り得るため、
逐次実行との比較は完全一致ではなく、共通header時刻での許容差を用います。
負荷が高いPCでは`--ekf-workers 2`または`3`へ下げてください。

## 結果を変えない追加短縮

低帯域入力の抽出、VO/EKF/描画のcache分離、wheel＋IMUの条件間共有を既定で使います。
再生倍率・センサ周波数・推定モデル・ゲート閾値は、この短縮のためには変更しません。

- **低帯域入力の抽出**: 各試行の`cache/lowband/`に、解析対象topicだけのSQLite bagを
  一度作ります。記録ns、topic、型、CDRバイト列、同時刻を含む順序をSHA256で
  元bagと照合し、不一致なら使用しません。OpenVINSの画像再計算は元bagを読みます。
  抽出bagのQoSメタデータは複製せず、既存の解析playerが定めるQoSをそのまま使います。
- **VO cache**: 元bag・校正ON/OFF・再生倍率・区間・初期校正ファイル・VO実装で判定します。
  方式の追加、磁気条件や描画の変更だけなら、OpenVINSは再実行しません。
- **EKF cache**: 実際の入力record・生成YAML・配線・区間・実装・実行backendで判定します。
  図だけの変更は数値計算を無効にしません。入力や有効parameterの変更は再計算します。
- **描画cache**: 数値出力bag、GNSS、地図、描画コードで判定します。
  図が欠けた場合も、完了済みの数値計算を使って描き直します。
- **wheel＋IMU共有**: VOを入力しない同一条件の結果は
  `sources/shared_wheel/mag_<条件>/<方式>/<解析日時>/`へ一度だけ保存します。
  VO別の重畳図はその下の`source_comparison/<VO条件>/`です。VO校正ON/OFFで
  wheel側を二重計算しません。磁気条件や方式が違う結果は共有しません。

完了印だけでなく、数値出力bagのmetadata・各partのサイズ/mtimeも確認します。
抽出cacheはDB・metadata自体のSHA256も毎回確認し、不完全なcacheは退避して作り直します。
元bagは保存後に変更しない前提です。元bagの変更検知はmetadata・サイズ・mtimeを使い、
巨大な元bag全体のhashを毎回読み直すことは避けています。
過去の広いfingerprintで保存したcacheを無理に流用せず、新しいkeyで一度再計算します。

```bash
# 通常のコマンドのまま短縮が有効。summary.jsonに各段階のcache利用状態を記録。
ros2 run pm_evaluation analyze_localization_batch \
  bags/rosbag2_2026_10_30-homearound \
  --bag rosbag2_2026_10_30-15_09_03 --stages comparison sources \
  --modes separated_offroad --vio-variants online-cam-calib_on online-cam-calib_off --magnetic on

# 抽出を介さず元bagを読む検証用オプション。
# --no-resumeは完了済み数値cacheも再計算するが、同一実行内のwheel共有は維持。
ros2 run pm_evaluation analyze_localization_batch \
  bags/rosbag2_2026_10_30-homearound \
  --bag rosbag2_2026_10_30-15_12_04 --stages comparison \
  --vio-variants recorded --magnetic on --duration 35 --no-lowband-cache --no-resume
```

短時間の実bag試験では５方式の初回が約67秒、同条件の再実行が約11秒でした。
これは35秒区間・記録済みVOを使った条件の値で、全期間の速度保証ではありません。
再計算にはROS timer等の実行順による小差が残ります。cache利用時は数値bagを
そのまま再利用し、別の推定結果へ置換したり軌跡をGNSSへfitしたりしません。

## コンソールの条件表示

各行に`[VO=online-cam-calib_on, magnetic=on]`のようにVO校正と磁気補正の
条件を表示します。VO再実行開始・失敗時もVO条件を表示します。
`online-cam-calib_on/off`は外部パラメータ`calib_cam_extrinsics`のON/OFFであり、
全てのオンライン校正機能を一括変更する指定ではありません。
旧CLI名`online_on/off`も受け付け、新名へ正規化します。新規の出力ディレクトリ・
summaryには新名を使用します。既存結果は改名しません。旧名で作成済みのcacheは
新名の計算へ自動移行しないため、初回は再計算になる場合があります。

## 図中の欠落・入力状態の注記

XY全体図・GNSS範囲図・z図・状態図に注記欄を設けます。図中文字は既存の描画環境で
文字化けしないよう英語とし、全文は同じ場所の`plot_annotations.json`にも保存します。

- `PLOT MISSING`: 描画予定だった系列が空、または有限値の描画対象がない。
- `INPUT NOT OBSERVED`: 当該EKFの生成YAMLで指定した入力topicが記録上見つからない。
  未記録と未配信は区別できないため、これだけではセンサ故障と断定しない。
- `INPUT ENDS EARLY`: 入力の最終headerが参照出力より2秒以上早い。
- `INPUT GAP`: 入力headerの間隔が1秒および正の間隔中央値の3倍を超える。高周波入力の小さい欠落を網羅する検査ではない。
- `DIAGNOSTIC`: OpenVINS・adapter・ゲートのWARN/ERROR状態が観測された。
  起動直後のinitializingやゲート安定化待ちも含むので、全期間の故障を意味しない。
- `INPUT SUSPECT/INVALID`: 観測されたOdometry速度が3 m/s超または非有限。前者は要確認の目安で、故障確定ではない。
- `OUTPUT WARNING/INVALID`: 推定速度3 m/s超、または不正な共分散・速度が検出された。

wheel＋IMU条件ではVOを意図的に外した旨を記し、VO不在を異常としません。
記録結果の図は入力融合の成否まで保証せず、記録から確認できる範囲で注記します。
VO再実行失敗では`vio_replay/<条件>/<解析日時>/failure_plot/`に、GNSSと欠落理由を載せた
XY/zの代替図を作り、索引からリンクします。欠落したlocal/globalを補間で捏造しません。
既存の描画cacheは描画コードの変更により再描画されます。既存の失敗条件の代替図は、
次のCLI実行で該当条件が処理された時に生成します。

## VO失敗時のwheel＋IMU比較

`sources`を選択している場合（既定で選択）、VOの初期化失敗・再実行失敗・
初期校正ファイル不足・記録raw不在でも、wheel＋IMUは元bagから独立に計算します。
VOを必要とする履歴再現・方式比較は実行せず、失敗状態をsummaryへ残します。
VO失敗をwheel側の成功で隠さず、CLI全体の終了コードは部分失敗を示す非0のままです。

`source_comparison/<VO条件>/`にはwheel＋IMU・GNSS・地図の図を残し、VO線が
ない理由を図中に注記します。校正ON/OFFの両方が失敗しても、同一wheel計算は
共有します。`--wheel-per-mode`を指定した場合も同様に各方式のVO除外条件を実行します。

## 解析directoryの日時名

新規のVO・EKF・低帯域cache・共有再生条件のdirectoryは、ハッシュ文字列ではなく
ローカルの解析日時`YYYYMMDD_HHMM`を使います。同じ親directoryで同じ分に複数の
結果を作る場合は`_02`、`_03`を付けます。内部識別用のkeyは
`analysis_identity.json`へ保存し、ディレクトリ名から計算条件を推測しません。

```text
comparison/online-cam-calib_on/mag_on/separated_offroad/20261103_2045/
sources/shared_wheel/mag_on/separated_offroad/20261103_2045/
```

完了済みの同条件を再利用する場合は、その結果を作成した日時directoryを使います。
毎回別の日時で数値計算をやり直す場合は`--no-resume`を指定します。
同一コマンド内のwheel共有は維持します。既存のハッシュ名directoryは改名・削除せず
残し、新しい日時形式へ自動移行はしません。そのため切替後の初回は再計算になります。

日時は開発コンテナのtimezone設定によらずJSTに揃えます。
VO失敗や数値不正で完了していない結果、出力bagが欠けた結果は再利用せず、
次回の試行を新しい日時directoryへ保存します。前回の失敗結果はそのまま残します。
完了済みだが速度warningがある結果は通常実行では再利用します。条件を変えず
試行差を調べるときは`--no-resume`を使ってください。

## 再生を２回へ集約する別版

既存コマンドを残したまま、`analyze_localization_batch_twopass`を追加しました。
[２回再生版の手順・負荷・制約](README_localization_batch_twopass.md)を参照してください。

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
