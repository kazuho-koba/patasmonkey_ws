#!/usr/bin/env bash
# Consoleと同じcontainer/ROS_DOMAIN_ID/表示設定を使ってプランナー単体を起動する。
set -euo pipefail
mission_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PM_GUI_LAUNCH_PACKAGE=pm_mission
export PM_GUI_LAUNCH_FILE=mission_planner.launch.py
export PM_GUI_LAUNCH_CONFIG="${PM_MISSION_CONFIG:-}"
exec bash "$mission_script_dir/pm_gui_container.sh"
