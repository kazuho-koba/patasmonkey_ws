#!/usr/bin/env bash
# Desktopからの起動ログを保存し、terminalがなくても失敗を通知する。
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${XDG_STATE_HOME:-${HOME}/.local/state}/patasmonkey"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/pm_gui_$(date +%Y%m%d_%H%M%S)_$$.log"

# ROS/container/domain設定は従来のwrapperと共通のenv fileを利用する。
# container、Robot Core、走行許可を自動で起動する処理は追加しない。
if "${SCRIPT_DIR}/pm_gui_container.sh" >"${LOG_FILE}" 2>&1; then
    exit 0
else
    launch_status=$?
fi
message="Patasmonkey GUIを正常に起動・終了できませんでした（終了コード ${launch_status}）。
開発containerが起動中か確認してください。
ログ: ${LOG_FILE}"
printf '%s\n' "${message}" >&2
if command -v zenity >/dev/null 2>&1; then
    zenity --error --title='Patasmonkey GUI' --text="${message}" --width=540 || true
elif command -v notify-send >/dev/null 2>&1; then
    notify-send --urgency=critical 'Patasmonkey GUI' "${message}" || true
fi
exit "${launch_status}"
