# pm_mission — Patasmonkey Mission Planner

Laptop用の独立したQt経路編集アプリと、UGV側の受領・保存nodeです。
**受領は走行開始ではありません。** Core、motor、cmd_vel、走行許可には触れません。
経由点間は直線で描画します。障害物回避・経路探索・走行可能性判定は行いません。

## 起動

開発PCホストから（起動済み`patasmonkey_foxy_dev`を使用）:

```bash
./scripts/pm_mission_container.sh
# ROS domain/configを指定する場合
PM_GUI_ROS_DOMAIN_ID=227 PM_MISSION_CONFIG=/workspaces/patasmonkey_ws/src/pm_mission/config/mission.yaml ./scripts/pm_mission_container.sh
```

ROS環境をsource済みのコンテナでは:

```bash
ros2 launch pm_mission mission_planner.launch.py
# または
ros2 run pm_mission mission_planner
```

`pm_gui`タイトル右側の **MISSION PLANNER** からも独立processとして起動します。
Consoleを閉じてもPlannerは継続し、どちらもCoreを自動起動しません。
GUI設定の`mission_planner.config`で起動するPlanner YAMLを指定できます。

UGV側の受信箱（Jetsonへ別途配置・ビルドし、ROS環境をsourceした後）:

```bash
ros2 launch pm_mission mission_receiver.launch.py
# 受信設定を指定
ros2 launch pm_mission mission_receiver.launch.py config:=/path/to/receiver.yaml
```

`pm_bringup/pm_core.launch.py`は受信箱を既定で起動します。
Coreが稼働していれば、motor enableとは独立して経路を受け取れます。
受信箱を別途手動起動する場合は、Core側を`use_mission_receiver:=false`にして重複を避けます。
設定変更には`mission_receiver_config:=/path/to/receiver.yaml`を使います。
受信箱単体でのsystemd登録は行っていません。
receiverは`rclpy`とYAML/modelだけを使用し、Qt windowを起動しません。

## GUIでの操作順

1. Operator Consoleの **MISSION PLANNER** を押すと、標準のPlanner launchで独立windowが開く。
2. **経路をロード** で既存YAMLを開くか、**新規経路** で地図クリックから作成する。
3. 地図上の点をdragしたり、一覧の座標・属性・待機秒・順番を編集する。
4. 必要に応じて **経路を保存**。ローカル保存とUGV送付は独立した操作。
5. **UGVへ読み込み（送付）** を押し、受信nodeの永続保存完了の応答を待つ。

現在のファイル、ローカルの未保存変更、UGVへの未送付/送付中/受領済みを別々に表示する。
受領済みの経路を編集すると、その新しい内容は未送付になる。
送付中に編集や別ファイルへの切替を行っても、応答は送付時の名前・revisionに対して表示する。
空の経路、START/GOALが未設定、受信nodeが未接続などの場合は送付ボタンを無効にし、
理由をtooltipに表示する。編集とローカル保存はUGV未接続でも利用可能。
受領確認timeout後は未確認状態を維持し、送付済みとは表示しない。

この段階の「UGVへ読み込み」は **検証・保存・受領応答** までを指す。
走行開始、経路追従、motor enableやcmd_vel送信は行わない。

## 編集

- 地図クリックで選択点の直後に追加（選択なしなら末尾）。最初はSTART、末尾追加はGOAL。
  末尾追加時、以前のGOALを通過に変更します。
- 経路点dragで位置変更、背景drag/右dragでpan。編集checkboxをOFFにすると左dragもpan。
- ホイールまたは＋／−でzoom。UGV位置・選択点・手入力座標へ移動できます。
- 一覧で緯度・経度・属性・待機秒・備考を編集し、削除・順序変更ができます。
- 属性: 通過、スタート、ゴール、一時停止、オペレータ確認。
- 送信時は2点以上、先頭START・末尾GOAL、中間にSTART/GOALなしを要求します。
  一時停止の秒数は0～86400秒。将来の走行executorが解釈する値で、現在は実行しません。
- 保存は編集途中でも可能。YAML schema version 1、WGS84、degree単位を明示します。
  未保存変更を新規作成・読み込み・終了時に確認します。

地図のonline取得・cache・offline XYZ tilesは`pm_ui_common`を通してConsoleと共用します。
設定は`config/mission.yaml`。cacheは`~/.cache/pm_gui/osm`。
offline directoryに`z/x/y.png`を配置するとInternetなしで表示できます。
未取得の地域には地図画像は出ませんが、gridと経路点の編集・保存は継続できます。
Web Mercatorの表示緯度上限は±85.05112878度です。
オンラインtileの一括先読みは行わず、画面範囲だけ取得します。
既存GNSS topic `/fix`（sensor_msgs/NavSatFix）で有効位置を表示します。
`/odometry/global`のx/yを緯度経度として利用しません。

## ROS interface / 責務

既存repositoryにミッション転送interfaceがないため`pm_msgs`へ追加しました。

| interface | 内容 |
|---|---|
| `pm_msgs/msg/MissionWaypoint` | 点ID、WGS84緯度経度、属性enum、待機秒、備考 |
| `pm_msgs/msg/Mission` | schema、mission ID、名前、revision、順序付き経路点 |
| `/pm/mission/upload` (`pm_msgs/srv/UploadMission`) | request ID、Mission → accepted、同じID、保存先、詳細 |

Laptopは編集・検証・送信・受領表示、Jetsonは検証・永続保存だけを担当します。
保存先は`config/receiver.yaml`の`output_directory`（既定は`~/.local/share/pm_mission/received`）。
receiverは一時fileへの書き込み・fsync・rename・directory fsyncの完了後にだけ成功応答します。
同じrequest ID・同じ内容の再送は同じ受領として扱い、異なる内容は拒否します。
送信中もQtはblockせず、timeout後は「UGV側で保存済みの可能性」を明示します。
同じdocumentの再送には同じrequest IDを使います。新たな編集には別IDを使います。
送信時snapshotのrevisionを受領表示するので、その後の編集と混同しません。
**走行開始・一時停止・破棄・executor接続・map frame変換は後続実装です。**

## ビルド

Ubuntu 22.04ホストではなくFoxy開発コンテナ内で実施します。

```bash
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
cd /workspaces/patasmonkey_ws
colcon build --packages-select pm_msgs pm_ui_common pm_mission pm_gui --symlink-install
# 実行時に初めてmain overlayをsource
source install/setup.bash
```

開発用receiverをLaptopで起動して受領を試す場合は、実機と混同しない専用domainを
Plannerとreceiverの両方に設定してください。実機Jetsonとのdomainは別にします。

### 道路内の位置を編集するための拡大

`map.max_zoom`は表示・編集の上限（既定26）、`map.tile_max_zoom`は
tile取得の上限（既定19）です。ホイール/＋でZ19を超えて拡大できます。
上限tileをQPainterで拡大し、地図画像・経路点・クリック/drag座標・
pan・scale barは同じ表示zoomで計算します。高zoomのtileは要求しません。
地図下部には画像拡大であることと倍率を表示します。
保存する緯度経度をtileのpixelに丸めることはありません。
画像拡大は地図情報の詳細や地図自体の位置精度を増やしません。
道路幅や地図とRTK位置のずれは別途確認が必要です。

### 反映範囲

Plannerを使うPCではpm_missionとpm_guiを更新する（pm_msgs/pm_ui_commonも導入済みであること）。
UGV側ではpm_missionとpm_bringupを更新する。
稼働中Coreにはファイル更新だけで受信nodeが追加されないため、次のCore起動で反映する。
今回のUploadMission/Mission/MissionWaypointのROS interfaceに変更はない。

### 起動時の位置・縮尺

標準設定は武蔵浦和駅付近（35.8461, 139.6466）、約1:20,000相当から開始する。
`map.initial_scale_denominator`とQtのlogical DPIから初期zoomを計算し、整数zoomの
タイルを端数倍率で描画する。96dpiでは地上分解能が約5.29m/pixelとなる。
実際の物理縮尺はモニターやOSのDPI設定に依存するため、地図上のスケールバーも参照する。
起動時にGNSSで別の場所へ移動しないよう、`map.center_on_first_fix: false`を標準とする。
UGVの位置表示は継続し、移動したい場合は「UGV位置へ」を使う。
既存経路をロードした場合の経路への移動は維持する。
[駅付近の座標参考](https://mapfan.com/spots/SCH%2CJ%2C6L)
