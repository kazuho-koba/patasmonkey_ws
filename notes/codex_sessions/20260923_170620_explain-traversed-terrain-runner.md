User prompt: この再生runnerというのはどうやって使うのですか？
機能の説明と、実際に使う際のコマンド例を示してください。

Codex final response: `run_traversed_terrain_replay.sh` は、指定rosbagに対して以下を一度に実行するrunnerです。

1. 最新の`separated_offroad` localizationを起動し、bag内のwheel/VIO/IMUから`odom -> base_link`と`/odometry/local`を再計算する。
2. OAK depthを現行terrain mapperへ渡して、rolling elevation mapとhazardを生成する。
3. 実走行軌跡を約25 cmごとに取り出し、それぞれの地点について約2 m手前で観測されたhazard mapを参照する。
4. 車体中心と、幅0.45 m・長さ0.55 mの暫定footprintについて、黒cell数・観測率・component原因を集計する。
5. JSON、CSV、PNG、launch/player/evaluator logを指定結果directoryへ保存し、runner終了時に自身が起動したROS processを停止する。

bagに記録された`/tf`や`/odometry/local`は再生しません。最新localizerがreplacement TFをpublishするためです。再生するのは`/wheel/odometry`、`/vio/odometry`、`/wit/imu`、`/tf_static`、`/oak/depth/image_raw`だけで、車両制御topicは扱いません。

開発PCでの実行例です。結果directoryは毎回新しい名前を指定してください。

```bash
cd /home/kazuho/Projects/patasmonkey_ws

docker exec -i --user 1000:1000 --env HOME=/home/developer \
  patasmonkey_foxy_dev bash -s -- \
  /workspaces/patasmonkey_ws/bags/rosbag2_2026_07_26-09_17_38 \
  /workspaces/patasmonkey_ws/src/pm_evaluation/results/terrain_hazard/rosbag2_2026_07_26-09_17_38/traversed_path_1x_repeat \
  < src/pm_evaluation/tools/run_traversed_terrain_replay.sh
```

約4分かかります。標準出力の最後に`summary.json`の内容が表示されます。保存物は以下です。

- `summary.json`: 全体集計
- `path_samples.csv`: 各通過地点のodom位置、参照した過去map時刻、中心・footprintのhazard値
- `traversed_hazard.png`: 軌跡上の中心黒判定と、走行距離に沿ったfootprint黒cell率・観測率
- `launch.log`、`player.log`、`evaluator.log`: 問題調査用ログ

他のROS実行系から分離したい場合は、`docker exec`に`--env TERRAIN_EVAL_DOMAIN_ID=98`を追加できます。既定は97です。
