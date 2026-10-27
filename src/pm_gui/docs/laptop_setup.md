# 野外実験用ノートPC：ホストとコンテナの作業分担

以下はGUIが既に起動できるノートPCへの更新手順。
修正版のソースをノートPCへ反映してから実施する。
既存のcontainer/workspaceを作り直す必要はない。
Jetson上のRobot Core、motor enable、bagにはこの作業で触れない。

| 作業 | 実行する場所 | 頻度 |
|---|---|---|
| ソース反映・Docker起動 | ノートPCのホスト | 更新時・必要時 |
| 日本語font導入 | ノートPCのコンテナ内・root | 古いコンテナでは1回 |
| ROS packageビルド | ノートPCのコンテナ内・一般ユーザー | ソース更新後 |
| GUI起動wrapperの実行 | ノートPCのホスト | 利用時 |
| Desktopアイコン登録 | ノートPCのホスト | 初回・workspace移動時 |

## 1. 日本語fontをコンテナに導入する

日本語が□になる場合、ホストにfontを入れてもコンテナのQtからは利用できない。
現在のDockerfileはfontconfig/fonts-noto-cjkを含むが、既存の古いimage/containerには後から反映されない。

**ホスト側：** GUIとPlannerを閉じ、コンテナが停止中なら起動する。

```bash
docker start patasmonkey_foxy_dev
docker exec -it --user root patasmonkey_foxy_dev bash
```

**開いたコンテナ内（root）：**

```bash
apt-get update
apt-get install -y fonts-noto-cjk fontconfig
fc-cache -f
fc-match :lang=ja
exit
```

`fc-match`がNoto Sans CJK JPなどの日本語fontを示すことを確認する。
最後のexitはホストのterminalへ戻る操作。
これはコンテナを再作成すると失われるため、その場合はfont導入済みimageを使う。
修正版の共通font処理はQtの日本語familyを選び、探索から漏れたsystem Noto TTCも直接登録する。
fontがない場合は起動ログへ導入案内を出す。wrapperはUTF-8 localeで起動する。

## 2. 修正版をビルドする

**ホスト側：** rootでビルドせず、通常の開発ユーザーで入り直す。

```bash
docker exec -it --user 1000:1000 --env HOME=/home/developer \
  patasmonkey_foxy_dev bash --noprofile --norc
```

**開いたコンテナ内（一般ユーザー）：**

```bash
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
cd /workspaces/patasmonkey_ws
colcon build --symlink-install --packages-select pm_ui_common pm_mission pm_gui
exit
```

build前にメインworkspaceのinstall/setup.bashはsourceしない。
上記はGUIが既に起動でき、依存packageをビルド済みの環境を前提とする。
sourceのpm_gui_container.shもノートPCへ反映すること。

## 3. GUIを起動する（ホスト側）

```bash
cd ~/Projects/patasmonkey_ws
PM_GUI_ROS_DOMAIN_ID=0 ./scripts/pm_gui_container.sh
```

wrapperがコンテナ内のROS overlayを順にsourceするため、ホストのROS環境をsourceする必要はない。
Jetsonと同じdomainを使用する。コンテナのhost networkとROS_LOCALHOST_ONLYの設定も一致させる。
ROS_DOMAIN_ID=0は現在の実機構成。mock/replay用の227と混同しない。

## 4. Desktopアイコン（ホスト側）

既存のアイコンはそのまま利用でき、今回の修正のための再登録は不要。
初回やworkspaceの配置先を変更したときだけ登録する。

```bash
cd ~/Projects/patasmonkey_ws
python3 scripts/install_pm_gui_desktop.py
```

必要に応じて`~/.config/patasmonkey/pm_gui.env`を以下の内容にする。
既存の設定がある場合は該当行だけ更新する。

```bash
PM_GUI_CONTAINER=patasmonkey_foxy_dev
PM_GUI_ROS_DOMAIN_ID=0
PM_GUI_CONFIG=/workspaces/patasmonkey_ws/src/pm_gui/config/gui.yaml
```

コンテナ起動後、DesktopのPatasmonkey GUIをダブルクリックする。
初回は右クリックで「起動を許可」が必要な場合がある。
アイコン自体はコンテナやRobot Coreを自動起動しない。
Desktop起動ログは`~/.local/state/patasmonkey/pm_gui_*.log`。

## Mission Plannerが起動したのに「起動失敗」と表示される不具合

PyQt5のQProcess.startDetachedの2引数版はboolを返す環境がある。
従来は戻り値を(started, pid)へ分解していたため、子process起動後にunpack例外が発生していた。
修正版はbool/tupleの両方を扱い、成功時に誤って警告しない。
ros2/PyQtの入れ替えやJetsonの変更は不要。GUIとPlannerを閉じて起動し直す。
