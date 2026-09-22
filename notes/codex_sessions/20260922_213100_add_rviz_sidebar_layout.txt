User prompt:
rviz2の設定を見るといま点群が全画面ででてしまうので、左側にサイドバーでFixed Frameを変更したり、新しいトピックを追加したりするようにできる設定にしてください。

Codex final response:
RViz設定を更新し、左側の常設サイドバーに以下を出すようにしました。

- `Displays`: `Global Options → Fixed Frame` の変更、既存displayのtopic変更、`Add`から新規topic/display追加
- `Tool Properties`
- `Views`
- `Selection` と `Time`

点群・OccupancyGridの表示は残したままです。`pm_perception`をFoxyコンテナ内で再build済みなので、前回と同じコマンドでRVizを起動すれば反映されます。

```bash
rviz2 -d "$(ros2 pkg prefix pm_perception)/share/pm_perception/rviz/terrain_mapping.rviz"
```

設定ファイル: `src/pm_perception/rviz/terrain_mapping.rviz`
