#!/usr/bin/env bash
# CoreとManagerを停止せず、失敗したbagの時刻確認unitだけ更新する。
set -euo pipefail
if [[ "$EUID" -ne 0 ]]; then
    echo 'sudo bash scripts/update_bag_clock_systemd.sh で実行してください' >&2
    exit 1
fi
deployment_workspace="${1:-/home/nvidia/patasmonkey_ws}"
deployment_share="$deployment_workspace/install/pm_robot_manager/share/pm_robot_manager/systemd"
for unit in pm-mission-bag.service pm-debug-bag.service; do
    unit_state="$(systemctl is-active "$unit" || true)"
    case "$unit_state" in
        inactive|failed) ;;
        *) echo "$unit が稼働中のため更新を中断します" >&2; exit 1 ;;
    esac
    test -f "$deployment_share/$unit"
done
deployment_backup="/var/backups/pm-robot-manager/bag-clock-$(date +%Y%m%d_%H%M%S)"
mkdir -p "$deployment_backup"
for unit in pm-mission-bag.service pm-debug-bag.service; do
    cp -a "/etc/systemd/system/$unit" "$deployment_backup/"
    install -m 0644 "$deployment_share/$unit" "/etc/systemd/system/$unit"
done
systemctl daemon-reload
systemctl reset-failed pm-mission-bag.service pm-debug-bag.service
# 今回bootで開始できなかったMissionだけ復旧する。Debugは起動しない。
systemctl start pm-mission-bag.service
systemctl is-active pm-mission-bag.service
echo "バックアップ: $deployment_backup"
