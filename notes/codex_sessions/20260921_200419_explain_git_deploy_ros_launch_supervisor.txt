User prompt
===========

github経由でjetsonでgit pullすればJetsonでも改善が反映されますか？

Codex final response
====================

はい。ただし、今回の変更を開発PCでcommitしてGitHubへpushした後に、Jetsonで同じbranchを`git pull`すれば、スクリプト側の改善は反映されます。現時点では変更はまだ未コミットなので、そのまま`git pull`しても反映されません。

Gitで反映されるものは以下です。

- `scripts/run_ros_launch.sh`（新規・実行権限あり）
- `scripts/start_pm.sh`の監督ラッパー利用とlaunchファイル名修正
- `scripts/README_ros_launch_supervisor.md`

Jetson側で`start-pm.service`が`/home/nvidia/patasmonkey_ws/scripts/start_pm.sh`を`ExecStart`しているなら、pull後の次回service起動からラッパーが使われます。スクリプト変更だけなら通常は`daemon-reload`不要です。

一方、`KillMode=control-group`と`TimeoutStopSec=30`はsystemd unitの設定であり、Git repository内の変更ではありません。これはJetsonのservice unitへ別途設定し、変更した場合だけ`sudo systemctl daemon-reload`が必要です。

またJetsonには、今回の実験で`pm_config/config/oak_d_s2/estimator_config1.yaml`を`track_frequency: 21.0`へ変更した未コミット差分があります。pull前に`git status`を確認してください。GitHub側で同じファイルを変更する場合は競合し得ます。
