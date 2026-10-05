#!/usr/bin/env bash
# CoreとManagerを停止せず、失敗したbagの時刻確認unitだけ更新する。
set -euo pipefail
if [[ "$EUID" -ne 0 ]]; then
    echo 'sudo bash scripts/update_bag_clock_systemd.sh で実行してください' >&2
    exit 1
fi
deployment_workspace="${1:-/home/nvidia/patasmonkey_ws}"
deployment_share="$deployment_workspace/install/pm_robot_manager/share/pm_robot_manager/systemd"
# is-activeの非zero終了は停止状態と通信エラーを区別できないため、
# showの成功を確認してLoadState/ActiveStateを個別に検証する。
read_unit_state() {
    local unit="$1" properties key value
    unit_load_state=''
    unit_state=''
    if ! properties="$(systemctl show "$unit" --property=LoadState --property=ActiveState)"; then
        echo "$unit の状態取得に失敗しました" >&2
        return 1
    fi
    while IFS='=' read -r key value; do
        case "$key" in
            LoadState) unit_load_state="$value" ;;
            ActiveState) unit_state="$value" ;;
            *) echo "$unit の状態取得結果が不正です: $key" >&2; return 1 ;;
        esac
    done <<< "$properties"
    case "$unit_load_state:$unit_state" in
        loaded:inactive|loaded:failed|not-found:inactive) ;;
        *) echo "$unit の更新を中断します: LoadState=$unit_load_state ActiveState=$unit_state" >&2
           return 1 ;;
    esac
}
for unit in pm-mission-bag.service pm-debug-bag.service; do
    read_unit_state "$unit"
    test -f "$deployment_share/$unit"
done
mkdir -p /var/backups/pm-robot-manager
# 同じ秒に再実行しても前回のバックアップを上書きしない。
deployment_backup="$(mktemp -d "/var/backups/pm-robot-manager/bag-clock-$(date +%Y%m%d_%H%M%S)-XXXXXX")"
for unit in pm-mission-bag.service pm-debug-bag.service; do
    if [[ -e "/etc/systemd/system/$unit" || -L "/etc/systemd/system/$unit" ]]; then
        cp -a "/etc/systemd/system/$unit" "$deployment_backup/"
    fi
    install -m 0644 "$deployment_share/$unit" "/etc/systemd/system/$unit"
done
systemctl daemon-reload
for unit in pm-mission-bag.service pm-debug-bag.service; do
    # reload後の実状態を再確認する。reset-failed自体の失敗は隠さない。
    read_unit_state "$unit"
    if [[ "$unit_state" == failed ]]; then
        systemctl reset-failed "$unit"
    fi
done
# 今回bootで開始できなかったMissionだけ復旧する。Debugは起動しない。
systemctl start pm-mission-bag.service
systemctl is-active pm-mission-bag.service
echo "バックアップ: $deployment_backup"
