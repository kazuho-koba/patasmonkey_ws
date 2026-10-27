"""ConsoleとMission Plannerで共用する日本語fontの選択処理。"""

import logging
from pathlib import Path

from PyQt5.QtGui import QFont, QFontDatabase


def configure_japanese_font(application, point_size=10):
    """QApplication生成後に呼び、選択したfamilyを返す。未導入時は警告する。

    Qt/fontconfigの探索先が外部ライブラリ等で限定された場合も、systemにある
    Noto CJKをアプリへ直接登録して利用する。font自体がない場合は自動取得せず、
    コンテナ内での導入を案内する。全widgetの継承元fontだけを設定する。
    """
    preferred = ('Noto Sans CJK JP', 'Noto Sans JP', 'IPAexGothic', 'IPAGothic', 'TakaoGothic')
    database = QFontDatabase()
    available = set(database.families())
    if not any(family in available for family in preferred):
        for name in ('NotoSansCJK-Regular.ttc', 'NotoSansCJK-Bold.ttc'):
            path = Path('/usr/share/fonts/opentype/noto')/name
            if path.is_file():
                QFontDatabase.addApplicationFont(str(path))
        available = set(database.families())
    family = next((name for name in preferred if name in available), None)
    if family is None:
        japanese = database.families(QFontDatabase.Japanese)
        family = japanese[0] if japanese else None
    if family:
        application.setFont(QFont(family, point_size))
    else:
        logging.getLogger(__name__).warning(
            'Japanese font not found in container. Install fonts-noto-cjk and fontconfig, '
            'run fc-cache -f, then restart the GUI.')
    return family
