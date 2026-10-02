#!/usr/bin/env bash
# Jetsonでbuild済みのunitを登録する。Coreとbagはenableだけ行い、今回は起動しない。
set -euo pipefail
if [[ "$EUID" -ne 0 ]]; then
    echo 'sudo bash scripts/install_robot_manager_systemd.sh で実行してください' >&2
    exit 1
fi
deployment_workspace="${1:-/home/nvidia/patasmonkey_ws}"
deployment_share="$deployment_workspace/install/pm_robot_manager/share/pm_robot_manager/systemd"
for unit in start-pm.service pm-mission-bag.service pm-debug-bag.service; do
    unit_state="$(systemctl is-active "$unit" || true)"
    case "$unit_state" in
        active|activating|deactivating)
            echo "$unit が稼働中のため登録を中断します。正常停止後に実行してください" >&2
            exit 1 ;;
    esac
done
visudo -cf "$deployment_share/pm-robot-manager.sudoers"
deployment_backup="/var/backups/pm-robot-manager/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$deployment_backup"
for unit in start-pm.service pm-robot-manager.service pm-mission-bag.service pm-debug-bag.service; do
    if [[ -f "/etc/systemd/system/$unit" ]]; then
        cp -a "/etc/systemd/system/$unit" "$deployment_backup/"
    fi
    if [[ -d "/etc/systemd/system/$unit.d" ]]; then
        cp -a "/etc/systemd/system/$unit.d" "$deployment_backup/"
    fi
    install -m 0644 "$deployment_share/$unit" "/etc/systemd/system/$unit"
done
if [[ -f /etc/sudoers.d/pm-robot-manager ]]; then
    cp -a /etc/sudoers.d/pm-robot-manager "$deployment_backup/sudoers.previous"
fi
install -m 0440 "$deployment_share/pm-robot-manager.sudoers" /etc/sudoers.d/pm-robot-manager
visudo -cf /etc/sudoers.d/pm-robot-manager
systemctl daemon-reload
systemctl reset-failed start-pm.service
systemctl enable start-pm.service pm-robot-manager.service pm-mission-bag.service
# missionはboot targetから独立して開始する。GUIのCore startには連動しない。
# 更新したManagerのコードを反映する。Coreとbagは起動しない。
systemctl restart pm-robot-manager.service
systemctl is-enabled start-pm.service pm-robot-manager.service pm-mission-bag.service
systemctl is-active pm-robot-manager.service
echo "バックアップ: $deployment_backup"
echo 'Coreとbagは起動していません。次回boot時にCoreとMission bagが開始します。'
