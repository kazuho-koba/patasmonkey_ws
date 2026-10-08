"""独立起動可能なQtミッション編集window。ROS通信・地図・documentを分離する。"""
import argparse
import copy
import hashlib
import json
import os
import signal
import sys
import time
import uuid

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QFontDatabase
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog,
    QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMainWindow, QMessageBox,
    QPushButton, QSplitter, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)
from .backend import Events, MissionBackend
from .map_view import MissionMap
from .model import ATTRIBUTES, LABELS, MAX_POINTS, load_document, new_mission, save_document, validate


class MissionWindow(QMainWindow):
    """編集documentはQt threadだけが所有し、送信時に独立snapshotを作る。"""
    def __init__(self, config, backend, events):
        super().__init__()
        self.config = config
        self.backend = backend
        self.document = new_mission()
        self.path = None
        self.dirty = False
        self.selected = -1
        self.pending = None
        self.last_fingerprint = self.last_request_id = None
        self._first_fix = True
        self._refreshing = False
        self.resize(1250, 820)
        self.setStyleSheet(
            'QMainWindow, QWidget { background:#101820; color:#f0f3bd; }'
            'QPushButton { background:#1d2b36; padding:6px 10px; border:1px solid #91a6b2; }'
            'QPushButton:disabled { color:#91a6b2; }'
            'QLineEdit,QTableWidget,QDoubleSpinBox,QComboBox { background:#1d2b36; }'
            'QHeaderView::section { background:#344b5b; padding:4px; }')
        central = QWidget()
        root = QVBoxLayout(central)
        self.setCentralWidget(central)
        title = QLabel('PATASMONKEY / MISSION PLANNER')
        title.setStyleSheet('font-size:18px; font-weight:bold; color:#f4d35e;')
        root.addWidget(title)
        controls = QHBoxLayout()
        for label, callback in [('新規', self.new), ('読み込み', self.load),
                                ('保存', self.save), ('名前を付けて保存', lambda: self.save(True))]:
            button = QPushButton(label)
            button.clicked.connect(lambda checked=False, fn=callback: fn())
            controls.addWidget(button)
        self.name = QLineEdit(self.document['name'])
        self.name.setMaxLength(200)
        self.name.textEdited.connect(self.rename)
        controls.addWidget(self.name, 1)
        self.send_button = QPushButton('UGVへ送信（保存のみ）')
        self.send_button.clicked.connect(self.send)
        controls.addWidget(self.send_button)
        root.addLayout(controls)
        self.connection = QLabel('受信nodeを検索中')
        root.addWidget(self.connection)
        self.map = MissionMap(config['map'])
        self.map.add_point.connect(self.add)
        self.map.move_point.connect(self.move)
        self.map.selection_changed.connect(self.select)
        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(['番号', '緯度', '経度', '属性', '待機秒', '備考'])
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setSelectionMode(QTableWidget.SingleSelection)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(5, QHeaderView.Stretch)
        self.table.cellChanged.connect(self.edit_cell)
        self.table.itemSelectionChanged.connect(self.table_selection)
        split = QSplitter(Qt.Vertical)
        split.addWidget(self.map)
        split.addWidget(self.table)
        split.setSizes([520, 200])
        split.setChildrenCollapsible(False)
        root.addWidget(split, 1)
        map_controls = QHBoxLayout()
        self.edit = QCheckBox('クリック追加／点drag移動')
        self.edit.setChecked(True)
        self.edit.toggled.connect(lambda enabled: setattr(self.map, 'edit_mode', enabled))
        map_controls.addWidget(self.edit)
        for label, callback in [('＋', lambda: self.map.set_zoom(self.map.zoom+1)),
                                ('−', lambda: self.map.set_zoom(self.map.zoom-1)),
                                ('UGV位置へ', self.center_vehicle),
                                ('選択点へ', self.center_selected)]:
            button = QPushButton(label)
            button.clicked.connect(lambda checked=False, fn=callback: fn())
            map_controls.addWidget(button)
        self.latitude = QDoubleSpinBox()
        self.longitude = QDoubleSpinBox()
        for spin, limit in ((self.latitude, 85.05112878), (self.longitude, 180)):
            spin.setDecimals(7)
            spin.setRange(-limit, limit)
            spin.setValue(self.map.center[0 if spin is self.latitude else 1])
            map_controls.addWidget(spin)
        center = QPushButton('指定座標へ')
        center.clicked.connect(lambda: self.map.set_center(self.latitude.value(), self.longitude.value()))
        map_controls.addWidget(center)
        root.addLayout(map_controls)
        edit_controls = QHBoxLayout()
        for label, callback in [('選択点を削除', self.delete), ('上へ', lambda: self.reorder(-1)),
                                ('下へ', lambda: self.reorder(1))]:
            button = QPushButton(label)
            button.clicked.connect(lambda checked=False, fn=callback: fn())
            edit_controls.addWidget(button)
        edit_controls.addWidget(QLabel('点は線分で接続。背景dragでpan／ホイールでzoom。'))
        root.addLayout(edit_controls)
        self.status = QLabel('経路を作成してください。送信しても走行は開始しません。')
        self.status.setWordWrap(True)
        root.addWidget(self.status)
        events.position.connect(self.position)
        events.receipt.connect(self.receipt)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(500)
        self.refresh()

    def changed(self):
        self.document['revision'] += 1
        self.dirty = True
        self.refresh()

    def refresh(self):
        """table再構築中のsignalを抑止し、選択点とdocument変更を分離する。"""
        self._refreshing = True
        self.table.blockSignals(True)
        points = self.document['waypoints']
        self.table.setRowCount(len(points))
        for row, point in enumerate(points):
            values = [str(row+1), '{:.8f}'.format(point['latitude']),
                      '{:.8f}'.format(point['longitude']), '', str(point['pause_sec']), point['note']]
            for column, value in enumerate(values):
                if column == 3:
                    combo = QComboBox()
                    combo.addItems(LABELS)
                    combo.setCurrentIndex(ATTRIBUTES.index(point['attribute']))
                    combo.currentIndexChanged.connect(lambda index, r=row: self.attribute(r, index))
                    self.table.setCellWidget(row, column, combo)
                else:
                    item = QTableWidgetItem(value)
                    if column == 0:
                        item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                        item.setToolTip(point['waypoint_id'])
                    self.table.setItem(row, column, item)
        if 0 <= self.selected < len(points):
            self.table.selectRow(self.selected)
        self.table.blockSignals(False)
        self._refreshing = False
        self.map.set_route(points, self.selected)
        self.setWindowTitle(('＊ ' if self.dirty else '')+self.document['name']+' / Mission Planner')

    def rename(self, name):
        self.document['name'] = name
        self.document['revision'] += 1
        self.dirty = True
        self.setWindowTitle('＊ '+name+' / Mission Planner')

    def add(self, latitude, longitude):
        points = self.document['waypoints']
        if len(points) >= MAX_POINTS:
            self.fail('経路点数の上限です')
            return
        # 末尾追加時だけ旧GOALを通過へ変更し、新しい末尾をGOALにする。
        # 属性を手動変更した点や途中挿入の属性は勝手に変更しない。
        insert = self.selected+1 if self.selected >= 0 else len(points)
        attribute = 'start' if not points else 'goal' if insert == len(points) else 'pass'
        if insert == len(points) and points and points[-1]['attribute'] == 'goal':
            points[-1]['attribute'] = 'pass'
        points.insert(insert, {'waypoint_id': str(uuid.uuid4()), 'latitude': latitude,
                               'longitude': longitude, 'attribute': attribute,
                               'pause_sec': 0.0, 'note': ''})
        self.selected = insert
        self.changed()

    def move(self, index, latitude, longitude):
        self.document['waypoints'][index].update(latitude=latitude, longitude=longitude)
        self.selected = index
        self.changed()

    def select(self, index):
        self.selected = index
        self.table.selectRow(index)
        self.map.set_route(self.document['waypoints'], index)

    def table_selection(self):
        if self._refreshing:
            return
        self.selected = self.table.currentRow()
        self.map.set_route(self.document['waypoints'], self.selected)

    def attribute(self, row, index):
        if not self._refreshing:
            self.document['waypoints'][row]['attribute'] = ATTRIBUTES[index]
            self.changed()

    def edit_cell(self, row, column):
        if self._refreshing or column not in (1, 2, 4, 5):
            return
        candidate = copy.deepcopy(self.document)
        key = {1: 'latitude', 2: 'longitude', 4: 'pause_sec', 5: 'note'}[column]
        value = self.table.item(row, column).text()
        try:
            candidate['waypoints'][row][key] = value if key == 'note' else float(value)
            self.document = validate(candidate)
            self.changed()
        except (ValueError, TypeError, KeyError) as error:
            self.fail(str(error))
            self.refresh()

    def delete(self):
        if 0 <= self.selected < len(self.document['waypoints']):
            del self.document['waypoints'][self.selected]
            self.selected = min(self.selected, len(self.document['waypoints'])-1)
            self.changed()

    def reorder(self, direction):
        points = self.document['waypoints']
        index, target = self.selected, self.selected+direction
        if 0 <= index < len(points) and 0 <= target < len(points):
            points[index], points[target] = points[target], points[index]
            self.selected = target
            self.changed()

    def center_selected(self):
        if 0 <= self.selected < len(self.document['waypoints']):
            point = self.document['waypoints'][self.selected]
            self.map.set_center(point['latitude'], point['longitude'])

    def center_vehicle(self):
        if self.map.vehicle:
            self.map.set_center(*self.map.vehicle)
        else:
            self.fail('有効なGNSS位置がまだ届いていません')

    def position(self, latitude, longitude):
        self.map.vehicle = (latitude, longitude)
        if self._first_fix and not self.document['waypoints']:
            self.map.set_center(latitude, longitude)
            self.map.set_zoom(int(self.config['map'].get('vehicle_zoom', 17)))
        self._first_fix = False
        self.map.update()

    def discard_or_save(self):
        if not self.dirty:
            return True
        response = QMessageBox.question(self, '未保存の変更', '変更を保存しますか？',
                                         QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel,
                                         QMessageBox.Save)
        if response == QMessageBox.Cancel:
            return False
        return self.save() if response == QMessageBox.Save else True

    def new(self):
        if self.discard_or_save():
            self.document = new_mission()
            self.path = None
            self.dirty = False
            self.selected = -1
            self.name.setText(self.document['name'])
            self.refresh()

    def load(self):
        if not self.discard_or_save():
            return
        path, _ = QFileDialog.getOpenFileName(self, 'ミッション読み込み',
                                             os.path.expanduser(self.config['mission_directory']),
                                             'Mission YAML (*.yaml *.yml)')
        if not path:
            return
        try:
            document = load_document(path)
            self.document, self.path, self.dirty, self.selected = document, path, False, -1
            self.name.setText(document['name'])
            self.refresh()
            if document['waypoints']:
                point = document['waypoints'][0]
                self.map.set_center(point['latitude'], point['longitude'])
                self.map.set_zoom(int(self.config['map'].get('vehicle_zoom', 17)))
            self.status.setText('読み込み: '+path)
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
            self.fail(str(error))

    def save(self, save_as=False):
        path = self.path
        if path is None or save_as:
            default = os.path.join(os.path.expanduser(self.config['mission_directory']), 'mission.yaml')
            path, _ = QFileDialog.getSaveFileName(self, 'ミッション保存', path or default,
                                                 'Mission YAML (*.yaml)')
        if not path:
            return False
        if not path.endswith(('.yaml', '.yml')):
            path += '.yaml'
        try:
            save_document(path, self.document)
            self.path, self.dirty = path, False
            self.refresh()
            self.status.setText('保存: '+path)
            return True
        except (OSError, ValueError, TypeError, KeyError) as error:
            self.fail(str(error))
            return False

    def send(self):
        if self.pending:
            return
        try:
            document = validate(self.document, True)
            fingerprint = hashlib.sha256(json.dumps(document, sort_keys=True).encode()).hexdigest()
            if fingerprint != self.last_fingerprint:
                self.last_request_id = str(uuid.uuid4())
                self.last_fingerprint = fingerprint
            self.pending = (self.last_request_id, time.monotonic(), document['revision'], document['name'])
            self.backend.submit(document, self.last_request_id)
            self.status.setText('UGV側の受領・保存完了を待っています…')
        except (ValueError, TypeError, KeyError, RuntimeError) as error:
            self.pending = None
            self.fail(str(error))
        self.poll()

    def receipt(self, request_id, accepted, detail):
        if self.pending is None or request_id != self.pending[0]:
            return
        revision, mission_name = self.pending[2], self.pending[3]
        self.pending = None
        self.status.setText(('受領済み: ' if accepted else '受領失敗: ')+mission_name
                            +' / revision '+str(revision)+'\n'+detail)
        self.status.setStyleSheet('color:'+('#7ac943' if accepted else '#ee6352'))
        self.poll()

    def poll(self):
        ready = self.backend.client.service_is_ready()
        self.connection.setText(('受信node接続済み' if ready else '受信node未接続')+' / '+self.config['upload_service'])
        if self.pending and time.monotonic()-self.pending[1] > float(self.config['upload_timeout_sec']):
            self.pending = None
            self.status.setText('受領確認timeout。UGV側で保存済みの可能性があります。同じ経路は安全に再送できます。')
            self.status.setStyleSheet('color:#ff9f1c')
        self.send_button.setEnabled(ready and self.pending is None)

    def fail(self, message):
        self.status.setText(message)
        self.status.setStyleSheet('color:#ee6352')

    def closeEvent(self, event):
        if self.discard_or_save():
            event.accept()
        else:
            event.ignore()


def main(args=None):
    """CLI/launchから起動し、QtとROS executorの終了を順序立てて行う。"""
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=os.path.join(
        get_package_share_directory('pm_mission'), 'config', 'mission.yaml'))
    parsed, ros_args = parser.parse_known_args(args)
    with open(os.path.expanduser(parsed.config), encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    application = QApplication(sys.argv[:1])
    families = set(QFontDatabase().families())
    if 'Noto Sans CJK JP' in families:
        application.setFont(QFont('Noto Sans CJK JP', 10))
    rclpy.init(args=ros_args)
    events = Events()
    backend = MissionBackend(config['ros'], events)
    window = MissionWindow(config, backend, events)
    signal.signal(signal.SIGINT, lambda *_: application.quit())
    signal.signal(signal.SIGTERM, lambda *_: application.quit())
    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(200)
    window.show()
    try:
        return application.exec_()
    finally:
        backend.close()
