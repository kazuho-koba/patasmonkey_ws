# OAK-D confidenceの記録・解析・閾値再処理

ROS 2 Foxy / DepthAI 2.x向け。EEPROMは書換えない。古いbagのconfidenceを復元しない。

## mission記録対象

| topic | 内容 |
|---|---|
| `/oak/stereo/confidence/image_raw` | mono8。0が高信頼、255が低信頼。stereo rectified座標 |
| `/oak/stereo/disparity/image_raw` | mono8／16UC1。既存StereoDepthのRGB align設定における出力。confidenceと同一座標ではない |
| `/oak/diagnostics/confidence_frame` | sequence、device/host timestamp等 |
| `/oak/diagnostics/disparity_frame` | 同上 |
| `/oak/stereo/recording_snapshot` | JSON：EEPROM、K/D、rectification、外部変換[cm]、pipeline、initialConfig、実効outConfig、SDK version/commit、MX ID |

通常depthが採用した同一device sequenceの診断画像だけをpublishする。stampはそのdepthからコピーし、元packetの時刻はmetadataに残す。後着packetを待って通常depthを遅らせず、20frame上限のcacheで照合する。別sequenceで欠測を補完しない。sequence gapには通常depthの20→約10 Hz選択も含まれる。

snapshotはtransient-local＋5秒再送。mission recorderは`provenance/oak_stereo_snapshot.json`にも保存する。`out_config: null`は実効設定未取得を意味する。独立したfirmware binary hashは未取得で、SDK version/commitを同梱firmware識別の根拠として保存する。既存missionに含まれる左右raw画像とmetadataも再処理に必要。

coreの引数は`oak_publish_depth_confidence:=true`、`oak_confidence_threshold:=240`が既定。前者をfalseにすると追加診断のXLink／publish／snapshotを停止する。閾値0..255は小さいほど厳格で、起動時に設定する。VOやdepthの既存topic・生成アルゴリズムは変更しない。

## カメラなしの分布解析

ビルド後、Foxy→外部workspace→メインworkspaceの順にsourceする。

```bash
ros2 run pm_evaluation analyze_depth_confidence \
  /workspaces/patasmonkey_ws/bags/<新しいbag> \
  --output /tmp/confidence_distribution
```

MCAP／SQLite対応で`metadata.yaml`が必要。`summary.json`、`histograms.csv`、`confidence_histograms.png`、`frames.csv`、校正snapshotを保存。画素重みの平均・median・p95・p99・maxと256bin分布を出す。デフォルトはnative confidence全画素のみで、無効depthに対応するconfidenceも含む。

## ground採用画素と追加マスクの近似診断

既存mapperのbag再評価時に`forensic_output_dir`を指定し、`cells.csv`を得る。forensic ROI／targetsによる範囲限定がある。新たなconfidence購読や計算を実行中mapperへ追加していない。

```bash
ros2 run pm_evaluation analyze_depth_confidence \
  /workspaces/patasmonkey_ws/bags/<新しいbag> \
  --output /tmp/confidence_ground \
  --ground-cells /tmp/mapper_forensic/cells.csv \
  --allow-unvalidated-geometry \
  --thresholds 200 220 240 --save-masked-depth
```

`last_accepted_ground_input_stamp_ns/pixel_u/pixel_v`を読み、snapshotに繰返し現れる採用sourceを一意化する。未受理のframe最小z候補を採用groundと数えない。分布は**CSVの診断ROIに現れた採用source**で、全mapの全採用点ではない。

`accepted_ground_confidence.csv`に撮像stamp、depthのu/v、深度[mm]、対応confidenceを保存する。元forensic情報へjoinし、ground置換や外れ値との関係を調べられる。summaryの`ground_requested/ground_mapped`、depth対応率、metadata欠測／sequence不一致も確認する。

### 未検証の幾何対応を明示する

confidenceにはRGB alignment／LR check／後処理が適用されない。同じ640×400でも同一pixelではない。解析はRGB depthから3Dを戻し、RGB→right・rectification・投影をしてnearest confidenceを読む。ただしrightのvirtual K、confidenceの基準camera、RGB depthの歪み／cropのSDK処理は**実機未検証**。明示opt-inが必要で、出力は`geometry_validated=false`。

`--depth-pixels-rectified`はRGB depth画素を歪みなしとする別仮説で、SDK warp確認後に選択する。対応不能は-1で最高信頼0と区別する。欠測metadata・別sequenceを最近傍stampで補わない。

`approx_threshold_*/<stamp>.npz`は追加マスクの**近似結果**。対応済みconfidence>thresholdの画素だけ0にする。対応不能画素は元depthを保持し、`correspondence_valid=false`とする。撮影時に閾値を変えた最終depthの完全再現ではない。

## 後日OAK接続後：同じStereoDepthによる再処理

以下はカメラ使用を伴うため、接続・実施の指示後に行う。driverとの同時利用不可。モーターや他センサを起動する必要はない。

```bash
# 記録時の閾値でbaselineを作り、元depthとの一致を検証する。
ros2 run pm_evaluation reprocess_depth_confidence \
  /workspaces/patasmonkey_ws/bags/<新しいbag> \
  --threshold 240 --output /tmp/confidence_baseline --allow-device

# 一致確認後に、同一入力・校正・SDKで厳しい閾値を比較する。
ros2 run pm_evaluation reprocess_depth_confidence \
  /workspaces/patasmonkey_ws/bags/<新しいbag> \
  --threshold 200 --output /tmp/confidence_strict200 --allow-device \
  --baseline-report /tmp/confidence_baseline/summary.json
```

記録のMX IDとSDK versionを使用する。EEPROM JSONをpipelineへ渡すだけで実機EEPROMをflashしない。全左右rawペアを元のtimestamp付きで順番に供給し、先に20→10 Hzへ間引かない。元depthの採用sequenceとleft metadataから保存stampへ戻す。CPUでstereo matchingを再実装する方法ではない。

出力はdepth[mm]の`<stamp>.npy`、`frames.csv`、`summary.json`。baselineでは画素一致数、有効mask差、共通有効画素の平均／最大絶対差[mm]を確認する。baseline不一致時は厳格閾値の再処理を既定で拒否する。原因確認後に限定比較を続ける場合だけ`--allow-baseline-difference`を明示し、完全再現とは呼ばない。

bag開始前のtemporal状態、入力欠落、firmware、内部warp／rounding、保存対象外設定は再現差になりうる。再生成depthの差だけで誤差が改善したとは判断せず、有効率／ground／hazardも別に比較する。**実機再処理はまだ未試験**。

## 負荷と検証状態

追加ROS画像はdepth採用周期約10 Hz。640×400でconfidence＋8bit disparityなら約5.12 MB/s、disparityが16bitなら約7.68 MB/s（payloadのみ）。XLinkはmono周期約20 Hzで流れるため、USBに約10.24／15.36 MB/s追加されうる。message overhead・圧縮・実測負荷は別。

校正は起動時、outConfig JSON化は初回のみ。snapshotは5秒周期。固定cacheを使いPointCloud2／全画素履歴をruntimeへ追加しない。JetsonのCPU／USB影響は後日測定する。

コンテナbuild、人工幾何、padding/endian、同一sequence、ground点dedup、SDK設定round-trip、合成SQLite／MCAP bagのCLIを確認。カメラアクセス・EEPROM読取り・実機座標照合・再処理は未実施。
