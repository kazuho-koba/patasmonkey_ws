# pm_ui_common

pm_guiとpm_missionで利用する非同期XYZ地図tile取得・cache管理です。
pm_guiの既存map_tiles importは互換性のため再公開します。

## 日本語font

`fonts.configure_japanese_font(application)`をQApplication生成後に呼ぶ。
Noto Sans CJK JPを優先し、他の日本語対応familyへfallbackする。
Qtの探索先が限定されている場合はsystemのNoto TTCをアプリへ直接登録する。
font自体はコンテナへ`fonts-noto-cjk`と`fontconfig`を導入する。
未導入時は起動ログへ案内を出し、アプリ内で自動ダウンロードやapt実行は行わない。
