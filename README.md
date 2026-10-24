# patasmonkey
off-road autonomous driving software stack

## 共通設定の編集

`pm_config`のYAMLはリンク配置するため、既存ファイルの値変更は再ビルド不要です。
初回の移行・Jetsonへの反映手順は[共通設定のREADME](src/pm_config/README.md)を参照してください。
workspace直下の`colcon build`では、`colcon_defaults.yaml`によって
`--symlink-install`が既定で有効になります。設定反映には対象nodeの再起動が必要です。
