User prompt
===========

変更を一度githubへgit pushし、jetson側にgit pullしました。track_frequencyの変更などもcommit済みです。
jetson側で実施すべき事項について実施してください。

Codex final response
====================

Jetson側のGit反映と、sudo不要な確認は完了しました。

- HEAD: `98df0f9 modified launch scripts`
- `git status`: clean
- `scripts/run_ros_launch.sh`: 実行可能
- `scripts/start_pm.sh`: 新ラッパーを参照
- `track_frequency`: source/installとも`21.0`
- 新ラッパー経由で`pm_bag_shibetsu.launch.py --show-args`が正常に解決
- `start-pm.service`: inactive
- 対象ROS/OAK/OpenVINS/bag recorderプロセス: 残留なし

systemdの最終保護である`KillMode=control-group` drop-inだけは、Jetsonのsudoパスワードが非対話SSHへ渡されないため、こちらから作成できませんでした。パスワードを扱わずに回避はしていません。

Jetsonの対話SSH端末で次を実行してください。

```bash
sudo install -d -m 0755 /etc/systemd/system/start-pm.service.d

sudo tee /etc/systemd/system/start-pm.service.d/process-group-cleanup.conf >/dev/null <<'EOF'
[Service]
KillMode=control-group
TimeoutStopSec=30
EOF

sudo systemctl daemon-reload
systemctl show start-pm.service -p KillMode -p TimeoutStopUSec -p KillSignal
```

期待値は`KillMode=control-group`、`TimeoutStopUSec=30s`、既存の`KillSignal=2`です。この操作はserviceを起動・再起動せず、車両も動かしません。実行後に知らせてもらえれば、こちらからread-onlyで設定反映を確認します。

`start-pm.service`が`start_pm.sh`を起動する限り、Gitで導入済みのラッパーがSIGINTをlaunchと全子ノードへ送り、15秒で終了しない場合も同じprocess groupだけへSIGTERMを送ります。今回のような親launch終了後の孤児化を防止できます。`KillMode=control-group`は、それでも残ったcgroup内プロセスをsystemdが回収する二重の保護です。
