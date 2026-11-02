# Bag Inspector

操縦GUIのMISSION PLANNER横のBAG INSPECTORから起動できます。
単独起動は`ros2 launch pm_inspector bag_inspector.launch.py`です。
既定はJetson上での解析で、bag本文のコピー・移動・ROS再生は行いません。
PCでの解析は上部の実行機を「このPCで解析」に切り替えます。

## 構成と起動

解析はpm_evaluation.bag_inspection、Qt画面と遠隔受付はpm_inspectorです。
遠隔受付はRobot Manager/Coreとは独立し、要求時にだけ低優先度の解析子processを起動します。
実機デプロイは自動では行いません。Jetson側はpm_msgs・pm_evaluation・pm_inspectorを
build/sourceした後、次で受付を起動してください（Motor/Core/記録は起動しません）。

```bash
ros2 launch pm_inspector inspection_agent.launch.py \
  bag_root:=$HOME/patasmonkey_ws/bags result_root:=$HOME/.ros/pm_inspector
```

PC側はpm_msgs・pm_evaluation・pm_inspector・pm_guiをbuildし、
GUIとJetsonでROS_DOMAIN_IDを一致させてください。
Foxyのbuildはコンテナ内で、ROS本体→外部workspaceの順にsourceし、
`colcon build --base-paths src --symlink-install --packages-select pm_msgs pm_evaluation pm_inspector pm_gui`
を実行します。実行時はメインworkspaceのinstallもsourceします。
ノートPCも同じ構成です。Qt・日本語フォントはpm_guiと共通です。

boot時の受付常駐用にsystemd/pm-inspector.serviceを同梱しています。
JetsonでunitのUser・Group・HOME・workspace・ROS_DOMAIN_IDを確認してから、
必要な場合だけ管理者が登録してください。既存Core/Managerのunitは変更しません。

```bash
sudo install -m 0644 install/pm_inspector/share/pm_inspector/systemd/pm-inspector.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now pm-inspector.service
```

受付だけならbag本体を読みません。解析時はJetsonのCPU/ディスクを使います。
走行中や記録中に自動解析する機能はありません。解析開始前に負荷確認dialogを出します。
遠隔受付が把握している記録中/保存処理中の同一bagは拒否し、走査中のファイル変更も検出します。
外部recorderの状態は把握できないため、必ず記録停止済みbagを選択してください。

## 一括解析・結果

一覧は指定directory直下のbagと子directoryを表示します。子directoryはダブルクリックで開きます。
複数bagにチェックし、解析開始します（最大100個、逐次実行）。
対象のrosbag2_***を一箇所のdirectoryに集約して指定する方法にも対応します。
一覧は200項目ごとで「続きを表示」が使えます。遠隔の参照範囲はbag_root配下だけです。
入力bagはmetadata.yamlとそこに記載されたMCAP/SQLiteの分割ファイルです。

保存整合性は実件数とmetadata・分割ファイル・利用可能なMCAP CRC・SQLite構造を確認します。
記録品質は固定窓Hz・最大record空白・record逆行と、既存品質解析の標準Header型について
headerの不正/重複/逆行/未来時刻（0.2秒許容）・最大間隔・記録遅延を集計します。
未対応Header型は未検査と表示し、期待Hzや真値位置を設定していない項目は合否を断定しません。
長時間bagはtopicあたり約1000窓までに窓幅を自動拡大し、結果に実際の窓幅を明記します。
画像decode・センサ内容の物理的妥当性・GNSS/位置精度判定は第一段階の対象外です。
全件読取と集計を共通化し、MCAPを検査項目ごとに再走査しません。

結果は実行機のresult_root/日時_UUID/へ保存され、bag原本は変更しません。
ローカルの場合は~/.ros/pm_inspector_localです。
各bagにreport.json・topics.csv・timeline.csv・completed.jsonを作成し、バッチにはjob.jsonとlogを保存します。
画面の結果選択からbagを切り替え、topic行をクリックすると時間窓Hzのグラフが表示されます。
「過去の結果を開く」では同じ実行機に保存した直近100ジョブを再表示できます。
LANで転送するのは一覧・状態・集計結果だけです。結果本文は512 KiB以内に制限します。
遠隔ジョブは画面を閉じても継続します。中止ボタンは解析子processにだけSIGTERMを送り、
中途結果にはcompleted.jsonを生成しません。Core/記録/原本を操作しません。

解析エンジン単独で既存結果の再表示用JSONも作成できます。

```bash
python3 -m pm_evaluation.bag_inspection --bag /path/to/stopped_bag \
  --output /path/outside_bag/new_result --window-sec 10
```

GUIのservice名・初期directory・窓幅・通信timeoutはconfig/inspector.yamlで変更できます。
serviceはpm_msgs/InspectBagsのoperation=list/start/status/cancel/result/history/loadを使い、payloadはJSONです。
任意shell実行やファイルダウンロード用の操作はありません。
ROSネットワークへの接続権を持つクライアントからの明示要求を受け付ける構成です。
