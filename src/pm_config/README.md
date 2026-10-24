# 共通YAML設定の編集とビルド

`pm_config`は実行コードを持たない`ament_cmake`パッケージ。
各launchが参照する`share/pm_config/config`の場所とYAMLの内容・パラメータ名は維持する。

## 既定のリンク配置

workspace直下の`colcon_defaults.yaml`で`build.symlink-install: true`を設定している。
workspace直下から実行する`colcon build`は、`--symlink-install`を省略しても
この設定を読み込む。ユーザーの`~/.colcon/defaults.yaml`の書換えは不要。
開発コンテナ・Jetsonに同じリポジトリを配置すれば、同じ既定値を利用できる。
`python3-colcon-defaults`拡張が必要で、開発コンテナでは既存の
`python3-colcon-common-extensions`に含まれている。

`install(DIRECTORY config ...)`により、既存YAMLはインストール先から
ソースへのファイル単位のリンクになる。`oak_d_s2`配下も同様に配置する。
ビルド・実行する環境と同じパスでソースを保持すること。
開発PCで生成した`build/`・`install/`をJetsonにコピーせず、Jetson上でビルドする。

## 初回のJetsonへの反映

この変更をリポジトリ単位でデプロイする。ルートの`colcon_defaults.yaml`、
`pm_config`の追加・変更・削除をすべて反映する。
旧`setup.py`・`setup.cfg`などは廃止し、`ament_package()`で登録を行う。

main workspaceのoverlayをまだsourceしていないビルド用シェルで、次を実行する。

```bash
cd ~/patasmonkey_ws
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
colcon build --packages-select pm_config
source install/setup.bash
```

workspace全体の`colcon build`でもよい。Foxy開発コンテナではworkspaceを
`/workspaces/patasmonkey_ws`、外部workspaceを`/workspaces/ros2_ws`と読み替える。
Ubuntu 22.04の開発ホストではFoxyをビルドしない。

旧Python形式の既存成果物がある開発コンテナでも、上記の通常リビルドで
既存YAMLのコピーがリンクへ置き換わることを確認している。
古い成果物や設定ディレクトリを手作業で削除する手順は不要。

配置先の確認例：

```bash
ros2 pkg prefix pm_config
ls -l install/pm_config/share/pm_config/config/vehicle_control.yaml
ls -l install/pm_config/share/pm_config/config/oak_d_s2/estimator_config1.yaml
```

リンク先が、その環境の`src/pm_config/config/...`となっていれば移行完了。

## 以降の設定変更

1. 実行するマシンの`src/pm_config/config/*.yaml`を編集する。
2. その設定を読み込むnode・launchを停止し、再起動する。

既存YAMLの値変更では再ビルド・再sourceは不要。`source`だけでは動作中の
nodeへパラメータを再読込しない。GUI・launchから明示的な値を後段で渡す場合は、
そちらが優先されることがあるため、変更対象の設定が実際に使用されているか確認する。

新しいYAMLの追加やインストール定義の変更時は、再びworkspace直下で
`colcon build --packages-select pm_config`を実行する。別ディレクトリからの
colcon実行では、このworkspaceの既定値ファイルは自動適用されない。

設定は`src`側を正本として管理する。`install`側をエディタで直接書き換えると、
エディタの保存方式によってはリンクを通常ファイルに置き換えてしまうので、
編集するパスは`src/pm_config/config`に統一する。
