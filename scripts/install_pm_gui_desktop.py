#!/usr/bin/env python3
"""ホストユーザーのDesktopとアプリ一覧にGUIランチャーを登録する。"""

import os
from pathlib import Path
import shutil
import subprocess


def main():
    """workspace位置を実行時に解決し、既存wrapperの設定を引き継ぐ。"""
    workspace = Path(__file__).resolve().parent.parent
    wrapper = workspace/'scripts/pm_gui_desktop.sh'
    icon = workspace/'src/pm_gui/desktop/pm-gui.svg'
    template = workspace/'src/pm_gui/desktop/pm-gui.desktop.in'
    for path in (wrapper, icon, template):
        if not path.is_file():
            raise RuntimeError('ランチャーに必要なファイルがありません: '+str(path))
    try:
        desktop = Path(subprocess.check_output(['xdg-user-dir', 'DESKTOP'], text=True).strip())
    except (OSError, subprocess.CalledProcessError):
        desktop = Path.home()/'Desktop'
    applications = Path(os.environ.get('XDG_DATA_HOME') or Path.home()/'.local/share')/'applications'
    # Execはshellコマンドではない。Desktop仕様の引用符/field codeとしてescapeする。
    executable = str(wrapper).replace('\\', '\\\\').replace('"', '\\"')
    executable = executable.replace('`', '\\`').replace('$', '\\$').replace('%', '%%')
    text = template.read_text().replace('@PM_GUI_WRAPPER@', executable)
    text = text.replace('@PM_GUI_ICON@', str(icon))
    targets = (applications/'pm-gui.desktop', desktop/'Patasmonkey GUI.desktop')
    for target in targets:
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text)
        target.chmod(0o755)
        print('登録: '+str(target))
    # GNOME Desktopでは実行権限とtrusted属性が必要。非GNOME環境では手動許可を案内する。
    if shutil.which('gio'):
        result = subprocess.run(['gio', 'set', str(targets[1]), 'metadata::trusted', 'true'],
                                capture_output=True, text=True)
        if result.returncode:
            print('Desktopで右クリックし「起動を許可」を選択してください: '+result.stderr.strip())
    if shutil.which('update-desktop-database'):
        subprocess.run(['update-desktop-database', str(applications)], check=True)


if __name__ == '__main__':
    main()
