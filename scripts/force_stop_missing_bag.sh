#!/usr/bin/env bash
# 保存先がないbagだけ強制停止する。稼働processを一旦止めて作成競合を防ぐ。
set -euo pipefail
[[ "$EUID" == 0 ]] || { echo 'sudo bashで実行してください' >&2; exit 1; }
profile="${1:-mission}"
case "$profile" in mission|debug) ;; *) echo 'missionまたはdebugを指定してください' >&2; exit 1;; esac
unit="pm-${profile}-bag.service"
status_file="${2:-/home/nvidia/.ros/pm_robot_manager/${profile}_bag_status.json}"
invocation_id="$(systemctl show "$unit" --property=InvocationID --value)"
[[ -n "$invocation_id" ]] || { echo '対象unitの起動IDがありません' >&2; exit 1; }
# SIGSTOPでrecorderとExecStopを停止し、判定中のdirectory新規作成を防ぐ。
# 拒否・検証エラー時は必ずSIGCONTで復帰させる。
systemctl kill --signal=SIGSTOP --kill-who=all "$unit"
resume=true
trap 'if [[ "$resume" == true ]]; then systemctl kill --signal=SIGCONT --kill-who=all "$unit"; fi' EXIT
python3 - "$status_file" "$invocation_id" "$profile" <<'PY'
import json, os, sys
from pathlib import Path
data = json.loads(Path(sys.argv[1]).read_text())
if data.get('invocation_id') != sys.argv[2] or data.get('profile') != sys.argv[3]:
    raise SystemExit('statusの起動ID/profileが一致しないため強制停止を拒否します')
output = data.get('output')
if not isinstance(output, str) or not output or not Path(output).is_absolute():
    raise SystemExit('保存先を確定できないため強制停止を拒否します')
# statの権限エラー等は不在と扱わない。親directory不在も明示的に確認する。
try:
    os.lstat(output)
except FileNotFoundError:
    pass
else:
    raise SystemExit('保存先が存在するため強制停止を拒否します: '+output)
print('保存先不在を確認。対象bag unitのみ強制停止します: '+output)
PY
systemctl kill --signal=SIGKILL --kill-who=all "$unit"
resume=false
systemctl stop "$unit"
# 強制停止由来のfailedだけを解除し、GUIが通常の開始待ちへ戻れるようにする。
final_state="$(systemctl show "$unit" --property=ActiveState --value)"
if [[ "$final_state" == failed ]]; then
    systemctl reset-failed "$unit"
    final_state="$(systemctl show "$unit" --property=ActiveState --value)"
fi
[[ "$final_state" == inactive ]] || { echo "停止後の状態が不正です: $final_state" >&2; exit 1; }
echo '強制停止しました。通常の記録開始待ちへ戻しました（保存成功ではありません）。'
