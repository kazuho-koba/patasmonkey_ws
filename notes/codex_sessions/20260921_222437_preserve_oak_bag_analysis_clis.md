User prompt

その解析スクリプトは使えそうなのでとっておいてください。

解析CLIはjetsonと開発PCのどちらにあるんですか？後者ならfoxyコンテナ、前者ならホスト環境で問題ないです。適切な方を選んでください。pm_evaluationをどちらで変更したかわかるように僕にも教えてください。

Codex final response

開発PC側の主リポジトリ `patasmonkey_ws` を変更しました。具体的には次の3ファイルです。

- `src/pm_evaluation/pm_evaluation/cli/analyze_oak_bag_diagnostics.py`
- `src/pm_evaluation/pm_evaluation/cli/summarize_tegrastats.py`
- `src/pm_evaluation/setup.py`

したがって、適切な実行・ビルド環境は開発PCの `patasmonkey_foxy_dev`（ROS 2 Foxy）コンテナです。Jetsonにはまだ反映していません。

2つのCLIを `pm_evaluation` の正式なconsole scriptとして追加し、Foxyコンテナで `colcon build --packages-select pm_evaluation` と各 `--help` を正常完了しました。

Jetsonへデプロイ後は、次のように実行できます。

```bash
ros2 run pm_evaluation analyze_oak_bag_diagnostics <bagディレクトリ>
ros2 run pm_evaluation summarize_tegrastats <tegrastats.log>
```
