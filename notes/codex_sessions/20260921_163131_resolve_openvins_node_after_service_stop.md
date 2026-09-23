Codex session record
Timestamp: 2026-09-21 16:31:31 JST

User prompt
===========
jetsonを立ち上げると自動的にUGVを動かすlaunchファイルが立ち上げるようになっているのですが、
それを　sudo systemctl stop start-pm.service　によって停止しても、

nvidia@pmjet1:~/ros2_ws/src/depthai_driver$ ros2 node list
/ov_msckf/run_subscribe_msckf

となってノードが残ってしまいます。
これの原因究明と解決を行ってください。

Codex final response
====================
原因を特定し、残留ノードを停止しました。現在Jetsonで`ros2 node list`を実行してもノードは表示されません。

原因は`start-pm.service`の停止失敗ではなく、開発PCのDockerコンテナ`patasmonkey_foxy_dev`に残っていた別のOpenVINSプロセスでした。Jetsonと開発PCが同じLAN上で同じ`ROS_DOMAIN_ID=0`を使っているため、開発PC側のノードがJetsonの`ros2 node list`にも表示されていました。

根拠は以下です。

- Jetsonの`start-pm.service`は`inactive (dead)`、`MainPID=0`、cgroupも空
- Jetson上に`run_subscribe_msckf`プロセスは存在しない
- systemd journalではJetson側OpenVINSへSIGINTが送られ、プロセス終了まで記録されている
- 表示されていたノードの`config_path`はJetsonの`/home/nvidia/...`ではなく、開発PCコンテナ固有の`/workspaces/patasmonkey_ws/...`
- 開発PCコンテナ内で、2026-09-20から残っていたparameter dump用のOpenVINS launchと`run_subscribe_msckf`を実際に確認

残留していた対象は以下のプロセスグループでした。

```text
ros2 launch ov_msckf subscribe.launch.py
run_subscribe_msckf
parameter dump用の親shell/docker exec
```

SIGINTでは既に停止処理が固着していて終了しなかったため、対象を確認したうえでそのプロセスグループだけにSIGTERMを送りました。その後、Jetson側の診断用ROS daemonを整理して再確認しました。

現在の状態は以下です。

```text
start-pm.service: inactive
MainPID: 0
Jetson上のOpenVINSプロセス: なし
開発PCコンテナ上の残留OpenVINS: なし
Jetsonのros2 node list: 空
```

車両serviceの再起動、ROS launchの起動、モーター指令は行っていません。Jetsonのコード・service unit・設定ファイルも変更していません。

なお、journalではJetson側OpenVINSが停止時に終了コード`-11`になっています。これは今回ノードが残って見えた直接原因ではありませんが、OpenVINSの終了処理に別の不具合がある可能性を示すため、後で切り分ける価値があります。

再発防止には、Jetson実機と開発PCで`ROS_DOMAIN_ID`を分離するのが最も確実です。例えばJetsonを`10`、開発PCを`20`にします。遠隔監視端末は対象機と同じdomainへ明示的に合わせます。これは既存の遠隔ROS監視方法に影響するため、今回は未変更です。希望があれば次に、service、SSH端末、Docker開発環境を含めたdomain分離を実装します。

今回の記録は[20260921_163131_resolve_openvins_node_after_service_stop.txt](/home/kazuho/Projects/patasmonkey_ws/notes/codex_sessions/20260921_163131_resolve_openvins_node_after_service_stop.txt)に保存しました。

Follow-up user prompt
=====================
この解決の前に、開発PC側のコンテナ環境でノードリストを見ても何も表示されていなかったのですが、それでもノードが起動していることがあるのですか？
またこの場合、どのようにして止めるべきか、コマンド例を教えてほしいです。
この問いに対する回答は20260921_163131_resolve_openvins_node_after_service_stop.txtの末尾に追記してください。

Follow-up Codex final response
==============================
はい、`ros2 node list`が空でもROSノードのプロセスが動いていることはあります。

`ros2 node list`が示すのはOS上のプロセス一覧ではなく、そのコマンドが参加しているDDS discovery範囲で発見できたROS graphです。そのため、ノードと確認コマンドの環境が次のいずれかで異なると、プロセスが存在しても一覧に出ません。

- `ROS_DOMAIN_ID`
- `ROS_LOCALHOST_ONLY`
- `RMW_IMPLEMENTATION`
- Fast DDSのprofile設定
- Dockerのnetwork namespaceまたはnetwork mode
- ROS 2 CLI daemonが起動されたときの環境
- sourceしているROS/workspace overlay

今回、開発PC側で一覧が空だった時点の環境変数とdaemon状態は保存されていないため、どの差が直接原因だったかまでは断定できません。一方、コンテナ内に以下の実プロセスが残っていたことと、そのノードがJetsonからparameter RPCへ応答したことは確認済みです。

```text
ros2 launch ov_msckf subscribe.launch.py
run_subscribe_msckf
```

したがって、停止確認では`ros2 node list`だけでなく、必ずプロセスも確認してください。

```bash
docker exec --user root patasmonkey_foxy_dev \
  pgrep -af '[r]un_subscribe_msckf|[s]ubscribe.launch.py'
```

親子関係とprocess groupも確認できます。

```bash
docker exec --user root patasmonkey_foxy_dev \
  ps -eo pid,ppid,sid,pgid,stat,lstart,args
```

通常は`ros2 launch`の親PIDへSIGINTを送ります。`<launch-pid>`は確認結果で置き換えてください。

```bash
docker exec --user root patasmonkey_foxy_dev \
  kill -INT <launch-pid>
```

数秒後、同じ`pgrep`で親と子が消えたことを確認します。SIGINTで終了しない場合は、PIDとPGIDをもう一度確認してから、対象launch専用のprocess groupへSIGTERMを送ります。

```bash
docker exec --user root patasmonkey_foxy_dev \
  ps -o pid,ppid,pgid,stat,args -p <launch-pid>,<node-pid>

docker exec --user root patasmonkey_foxy_dev \
  kill -TERM -- -<verified-pgid>
```

負の値はPIDではなくprocess group全体を意味するため、`<verified-pgid>`を確認せずに実行してはいけません。`pkill -f ros2`や`killall`は無関係なROSノードまで停止し得るため推奨しません。

停止後、必要ならROS 2 CLI daemonの表示を更新します。

```bash
docker exec --user 1000:1000 --env HOME=/tmp patasmonkey_foxy_dev \
  bash -lc 'source /opt/ros/foxy/setup.bash; ros2 daemon stop'
```

その後の`ros2 node list`でdaemonは必要に応じて再起動します。ただし、daemon再起動はプロセスを止める操作ではありません。先に`pgrep`で実プロセスを終了させる必要があります。

診断用launchを今後バックグラウンド実行する場合は、専用process groupを作り、正常停止後にタイムアウト付きでSIGTERMへ移行できる形にするのが安全です。

```bash
setsid ros2 launch ov_msckf subscribe.launch.py ... &
launch_pid=$!

# 診断処理

kill -INT -- "-$launch_pid"
for _ in 1 2 3 4 5; do
  kill -0 "$launch_pid" 2>/dev/null || break
  sleep 1
done
if kill -0 "$launch_pid" 2>/dev/null; then
  kill -TERM -- "-$launch_pid"
fi
wait "$launch_pid" || true
```

恒久的には、Jetsonと開発PCへ異なる`ROS_DOMAIN_ID`を割り当てると、片方のノードがもう片方の`ros2 node list`に混入することを防げます。

指定どおり、この回答を[20260921_163131_resolve_openvins_node_after_service_stop.txt](/home/kazuho/Projects/patasmonkey_ws/notes/codex_sessions/20260921_163131_resolve_openvins_node_after_service_stop.txt)の末尾へ追記しました。
