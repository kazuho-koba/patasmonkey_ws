User prompt: README、コメントについては日本語でお願いしたいです。その旨を修正の上、AGENTS.mdにも追記してもらえますか＞

Codex final response: 対応しました。

[AGENTS.md](../../AGENTS.md) に、README・docstring・コードコメントは日本語で書くことを明記しました。ROS名、API名、変数名、数式、定着した技術用語は英語表記を許容しています。

既存の `pm_perception`、`bag_clock_player.py`、`pm_bag_global_localization.launch.py` の今回実装・追記分についても、英語だったdocstringとコメントを日本語へ統一しました。READMEは既に本文が日本語だったため、不必要な書き換えはしていません。

`git diff --check`および対象Pythonファイルの`py_compile`は通過しています。コメント・文書のみの変更なのでROS buildは不要です。
