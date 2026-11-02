"""bagをコピーせず、選択した実行機の資源で解析するQt画面。"""
import argparse
import json
import os
from pathlib import Path
import queue
import threading
import time

from PyQt5.QtCore import Qt, QThread, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QPainter
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QLineEdit, QListWidget, QListWidgetItem, QDoubleSpinBox,
    QMessageBox, QFileDialog, QTableWidget, QTableWidgetItem, QHeaderView, QSplitter,
    QPlainTextEdit, QInputDialog)
import yaml
import rclpy
from pm_msgs.srv import InspectBags
from pm_inspector.jobs import Jobs
from pm_ui_common.fonts import configure_japanese_font


class Requests(QThread):
    """ROSの探索・応答待ちをQt threadから分離し、通信断でも画面を止めない。"""
    ready = pyqtSignal(str, object)
    failed = pyqtSignal(str)

    def __init__(self, service, timeout):
        super().__init__()
        self.service, self.timeout = service, timeout
        self.tasks = queue.Queue()
        self.stopping = threading.Event()
        self.local = Jobs(Path.home(), Path.home()/'.ros/pm_inspector_local')

    def run(self):
        node = rclpy.create_node('pm_inspector_client')
        client = node.create_client(InspectBags, self.service)
        try:
            while not self.stopping.is_set():
                try:
                    remote, operation, payload = self.tasks.get(timeout=.1)
                except queue.Empty:
                    continue
                try:
                    if not remote:
                        if operation == 'list':
                            path = Path(payload['root']).expanduser().resolve()
                            # PCのdirectory選択は任意パスを許容する。遠隔はagentのroot制限を適用。
                            self.local.root = Path('/')
                            payload = dict(payload, root=str(path))
                        answer = self.local.dispatch(operation, payload)
                    else:
                        deadline = time.monotonic()+self.timeout
                        while not client.service_is_ready():
                            if self.stopping.is_set() or time.monotonic() > deadline:
                                raise RuntimeError('Jetson解析受付が見つかりません。agent起動・ROS_DOMAIN_IDを確認してください')
                            client.wait_for_service(timeout_sec=.1)
                        request = InspectBags.Request()
                        request.operation, request.payload = operation, json.dumps(payload)
                        future = client.call_async(request)
                        while not future.done():
                            if self.stopping.is_set() or time.monotonic() > deadline:
                                future.cancel()
                                raise RuntimeError('通信待ちがtimeoutしました。実行要求の成否は状態更新で確認してください')
                            rclpy.spin_once(node, timeout_sec=.1)
                        response = future.result()
                        answer = json.loads(response.payload)
                        if not response.success:
                            raise RuntimeError(answer.get('error', '要求失敗'))
                    self.ready.emit(operation, answer)
                except Exception as error:
                    self.failed.emit(str(error))
        finally:
            self.local.close()
            node.destroy_node()


class Timeline(QWidget):
    """選択topicの固定時間窓Hzを描く。bagデータではなく集計値だけを保持する。"""
    def __init__(self):
        super().__init__()
        self.values, self.window, self.duration = [], 10., 0.
        self.setMinimumHeight(130)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor('#17212b'))
        painter.setPen(QColor('#e0e8ee'))
        if not self.values:
            painter.drawText(self.rect(), Qt.AlignCenter, 'トピックをクリックすると時間窓Hzを表示')
            return
        maximum = max(max(self.values), 1.)
        width, height = max(self.width()-90, 1), max(self.height()-55, 1)
        for i, value in enumerate(self.values):
            x = 65+int(i*width/len(self.values))
            size = max(1, int(width/len(self.values)))
            h = int(height*value/maximum)
            painter.fillRect(x, 25+height-h, size, h, QColor('#4bc8ad'))
        painter.drawText(5, 20, '{:.2f} Hz'.format(maximum))
        painter.drawText(65, self.height()-7, '0 s   /   窓 {:.1f} s   /   終端 {:.1f} s'.format(
            self.window, self.duration))


class Window(QMainWindow):
    """既定はJetson解析。ローカルと遠隔を切替えてもbag本文は転送しない。"""
    def __init__(self, config):
        super().__init__()
        self.setWindowTitle('PATASMONKEY / BAG INSPECTOR')
        self.resize(1200, 800)
        self.config, self.busy = config, False
        self.entries_signature = None
        self.next_offset = None
        root = QWidget()
        layout = QVBoxLayout(root)
        top = QHBoxLayout()
        self.mode = QComboBox()
        self.mode.addItems(['Jetsonで解析（既定・bag転送なし）', 'このPCで解析'])
        top.addWidget(self.mode)
        self.path = QLineEdit(config.get('remote_root', ''))
        self.path.setPlaceholderText('空欄はJetsonのbagルート。直接の子directoryを一覧表示')
        top.addWidget(self.path, 1)
        for text, function in [('参照', self.browse), ('一覧更新', self.list_bags), ('親へ', self.parent)]:
            button = QPushButton(text)
            button.clicked.connect(function)
            top.addWidget(button)
        layout.addLayout(top)
        self.list = QListWidget()
        self.list.itemDoubleClicked.connect(self.enter_directory)
        layout.addWidget(self.list, 2)
        controls = QHBoxLayout()
        for text, function in [('全bag選択', self.select_all), ('選択解除', self.clear_selection),
                               ('続きを表示', self.more)]:
            button = QPushButton(text)
            button.clicked.connect(function)
            controls.addWidget(button)
        controls.addWidget(QLabel('時間窓[s]'))
        self.window = QDoubleSpinBox()
        self.window.setRange(.1, 3600.)
        self.window.setValue(float(config.get('window_sec', 10.)))
        controls.addWidget(self.window)
        self.start_button = QPushButton('選択bagを解析')
        self.start_button.clicked.connect(self.start_analysis)
        controls.addWidget(self.start_button)
        self.cancel_button = QPushButton('解析中止')
        self.cancel_button.clicked.connect(lambda: self.send('cancel', {}))
        controls.addWidget(self.cancel_button)
        layout.addLayout(controls)
        self.status = QLabel('Jetsonのagentを起動して一覧更新してください。記録停止済みbagを選択します。')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.results = QComboBox()
        self.results.activated.connect(self.load_result)
        result_row = QHBoxLayout()
        result_row.addWidget(self.results, 1)
        history = QPushButton('過去の結果を開く')
        history.clicked.connect(lambda: self.send('history', {}))
        result_row.addWidget(history)
        layout.addLayout(result_row)
        self.table = QTableWidget(0, 8)
        self.table.setHorizontalHeaderLabels(['topic', '件数', '平均Hz', '最大空白[s]',
                                               'record逆行', 'header逆行', 'header重複', 'header不正/未来'])
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.table.cellClicked.connect(self.topic_result)
        layout.addWidget(self.table, 3)
        self.timeline = Timeline()
        layout.addWidget(self.timeline)
        self.details = QPlainTextEdit()
        self.details.setReadOnly(True)
        self.details.setMaximumHeight(90)
        layout.addWidget(self.details)
        self.setCentralWidget(root)
        self.worker = Requests(config.get('service', '/pm/inspector/request'),
                               float(config.get('request_timeout_sec', 10.)))
        self.worker.ready.connect(self.received)
        self.worker.failed.connect(self.failed)
        self.worker.start()
        self.mode.currentIndexChanged.connect(self.change_mode)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(2000)

    def send(self, operation, payload):
        if self.busy:
            return
        self.busy = True
        self.worker.tasks.put((self.mode.currentIndex()==0, operation, payload))

    def poll(self):
        if not self.busy:
            self.send('status', {})

    def list_bags(self):
        if self.busy:
            return
        self.listing_more = False
        self.send('list', dict(root=self.path.text(), offset=0))

    def more(self):
        if not self.busy and self.next_offset is not None:
            self.listing_more = True
            self.send('list', dict(root=self.path.text(), offset=self.next_offset))

    def browse(self):
        if self.mode.currentIndex() == 0:
            self.list_bags()
        else:
            path = QFileDialog.getExistingDirectory(self, 'bag集約directoryを選択')
            if path:
                self.path.setText(path)
                self.list_bags()

    def parent(self):
        if self.path.text():
            self.path.setText(str(Path(self.path.text()).parent))
            self.list_bags()

    def enter_directory(self, item):
        kind, path = item.data(Qt.UserRole)
        if kind == 'directory':
            self.path.setText(path)
            self.list_bags()

    def select_all(self):
        self.check_items(Qt.Checked)

    def clear_selection(self):
        self.check_items(Qt.Unchecked)

    def check_items(self, state):
        for i in range(self.list.count()):
            item = self.list.item(i)
            if item.data(Qt.UserRole)[0] == 'bag':
                item.setCheckState(state)

    def change_mode(self):
        if self.busy:
            # 結果の実行機を取り違えないよう、通信完了前の切替は戻す。
            self.mode.blockSignals(True)
            self.mode.setCurrentIndex(1-self.mode.currentIndex())
            self.mode.blockSignals(False)
            return
        self.path.setText(self.config.get('remote_root', '') if self.mode.currentIndex()==0
                          else str(Path(self.config.get('local_root', '~/patasmonkey_ws/bags')).expanduser()))
        self.list.clear()
        self.results.clear()
        self.entries_signature = None
        self.table.setRowCount(0)
        self.list_bags()

    def start_analysis(self):
        bags = [self.list.item(i).data(Qt.UserRole)[1] for i in range(self.list.count())
                if self.list.item(i).data(Qt.UserRole)[0]=='bag'
                and self.list.item(i).checkState()==Qt.Checked]
        if not bags:
            QMessageBox.information(self, 'bag選択', '解析するbagにチェックを付けてください')
            return
        text = '{}個のbagを{}で順番に全件走査します。\nCPU・ディスク負荷が発生します。記録中のbagは対象にしないでください。'.format(
            len(bags), 'Jetson' if self.mode.currentIndex()==0 else 'このPC')
        if QMessageBox.question(self, '解析開始', text) == QMessageBox.Yes:
            self.send('start', dict(bags=bags, window_sec=self.window.value()))

    def load_result(self, index=None):
        index = self.results.currentIndex() if index is None else index
        if index >= 0:
            self.send('result', dict(index=index))

    def topic_result(self, row, column):
        self.send('result', dict(index=self.results.currentIndex(), topic=self.table.item(row, 0).text()))

    def received(self, operation, answer):
        self.busy = False
        if operation == 'list':
            # offset=0要求時は前の一覧を置換する。
            if not getattr(self, 'listing_more', False):
                self.list.clear()
            self.listing_more = False
            self.path.setText(answer['root'])
            for kind, paths in [('directory', answer['directories']), ('bag', answer['bags'])]:
                for path in paths:
                    item = QListWidgetItem(('[DIR] ' if kind=='directory' else '')+Path(path).name)
                    item.setData(Qt.UserRole, (kind, path))
                    item.setToolTip(path)
                    if kind == 'bag':
                        item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
                        item.setCheckState(Qt.Unchecked)
                    self.list.addItem(item)
            self.next_offset = answer['next_offset']
            self.status.setText('一覧取得完了。directoryはダブルクリックで開けます。')
        elif operation == 'status':
            entries = answer.get('entries', [])
            progress = answer.get('progress', {})
            self.status.setText('{} / {} / {}/{} messages / {}'.format(answer['state'],
                answer.get('current', ''), progress.get('processed', '?'), progress.get('expected', '?'),
                answer.get('error', '') or progress.get('error', '')))
            self.start_button.setEnabled(answer['state'] != 'RUNNING')
            self.cancel_button.setEnabled(answer['state'] == 'RUNNING')
            signature = [(e['bag'], e['state']) for e in entries]
            changed = signature != self.entries_signature
            if changed:
                old = max(0, self.results.currentIndex())
                self.results.clear()
                for e in entries:
                    self.results.addItem(e['state']+' / '+Path(e['bag']).name)
                self.results.setCurrentIndex(min(old, len(entries)-1))
                self.entries_signature = signature
                if entries and entries[self.results.currentIndex()]['state']=='DONE':
                    self.load_result()
            if entries and changed:
                self.details.setPlainText('\n'.join(e.get('error', '') for e in entries if e.get('error'))
                                          or '解析結果: '+answer.get('job_directory', ''))
        elif operation == 'result':
            if 'statistics' in answer:
                self.timeline.values = answer['statistics']['window_hz']
                self.timeline.window = answer['window_sec']
                self.timeline.duration = answer['duration_sec']
                self.timeline.update()
                self.details.setPlainText(json.dumps({k:v for k,v in answer['statistics'].items()
                                                     if k!='window_hz'}, ensure_ascii=False))
            else:
                self.table.setRowCount(0)
                for topic, row in sorted(answer['topics'].items()):
                    i = self.table.rowCount()
                    self.table.insertRow(i)
                    values = [topic, row['count'], row['average_hz'], row.get('max_gap_sec'),
                              row.get('record_backwards'), row.get('header_backwards'),
                              row.get('header_duplicates'), '{}/{}'.format(
                                  row.get('header_invalid', '未対応'), row.get('header_future', '未対応'))]
                    for j, value in enumerate(values):
                        text = '--' if value is None else '{:.3f}'.format(value) if isinstance(value, float) else str(value)
                        self.table.setItem(i, j, QTableWidgetItem(text))
                self.table.resizeColumnsToContents()
                self.details.setPlainText('保存整合性: {} / 時刻異常件数: {}\n{}'.format(
                    '合格' if answer['integrity_verified'] else '不一致',
                    answer['timestamp_issue_count'], answer['note']))
        else:
            self.status.setText(answer.get('message', '要求完了'))
            if operation == 'start':
                self.table.setRowCount(0)
                self.timeline.values = []
                self.timeline.update()
            if operation == 'history':
                if not answer['jobs']:
                    QMessageBox.information(self, '解析履歴', '保存済みの解析履歴はありません')
                else:
                    name, accepted = QInputDialog.getItem(self, '解析履歴', '保存済みジョブ', answer['jobs'], editable=False)
                    if accepted:
                        self.send('load', dict(job=name))
            elif operation == 'load':
                self.entries_signature = None
                self.poll()

    def failed(self, error):
        self.busy = False
        self.status.setText('通信/操作エラー: '+error)

    def closeEvent(self, event):
        self.worker.stopping.set()
        self.worker.wait(12000)
        if self.worker.isRunning():
            event.ignore()
            return
        event.accept()


def main():
    from ament_index_python.packages import get_package_share_directory
    from rclpy.utilities import remove_ros_args
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=str(Path(get_package_share_directory('pm_inspector'))/'config/inspector.yaml'))
    args = parser.parse_args(remove_ros_args()[1:])
    config = yaml.safe_load(Path(args.config).read_text())
    os.environ.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE', str(
        Path(get_package_share_directory('pm_inspector'))/'config/control_dds_udp.xml'))
    rclpy.init()
    app = QApplication([])
    configure_japanese_font(app)
    window = Window(config)
    window.show()
    try:
        app.exec_()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
