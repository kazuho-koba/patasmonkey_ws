#!/usr/bin/env python3
"""Qt operator console。ROS executorは別threadで動作させる。"""

import argparse
import os
import signal
import sys
import threading

import rclpy
import yaml
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont, QFontDatabase
from PyQt5.QtWidgets import (
    QApplication, QGridLayout, QHBoxLayout, QLabel, QMainWindow, QMessageBox,
    QPushButton, QTabWidget, QVBoxLayout, QWidget,
)

from .panels import CameraPanel, LocalizationPanel, PALETTE, RecordingPanel, TelemetryPanel
from .ros_backend import RosBackend


class OperatorWindow(QMainWindow):
    """Robot Core管理と監視機能のtabを提供する。"""

    def __init__(self, backend, config):
        super().__init__()
        self.backend = backend
        self.config = config
        ui_config = config.get('ui', {})
        self.setWindowTitle(ui_config.get('window_title', 'Patasmonkey UGV Operator Console'))
        self.resize(*ui_config.get('window_size', [1120, 780]))
        self.setStyleSheet(
            'QMainWindow, QWidget { background:' + PALETTE['background'] + '; color:' + PALETTE['text'] + '; }'
            'QGroupBox { border:2px solid ' + PALETTE['muted'] + '; margin-top:10px; padding:8px; font-weight:bold; }'
            'QGroupBox::title { subcontrol-origin:margin; left:8px; padding:0 4px; color:' + PALETTE['blue'] + '; }'
            'QPushButton { background:' + PALETTE['panel'] + '; color:' + PALETTE['text'] + '; border:2px solid '
            + PALETTE['muted'] + '; padding:8px 14px; font-weight:bold; }'
            'QPushButton:disabled { color:' + PALETTE['muted'] + '; }'
            'QTabBar::tab { background:' + PALETTE['panel'] + '; padding:9px 18px; border:2px solid '
            + PALETTE['muted'] + '; } QTabBar::tab:selected { color:' + PALETTE['yellow'] + '; }'
            'QCheckBox { spacing:10px; padding:6px; }'
        )
        root = QWidget()
        layout = QVBoxLayout(root)
        title = QLabel('PATASMONKEY  /  OPERATOR CONSOLE')
        title.setStyleSheet('font-size:20px; font-weight:bold; color:' + PALETTE['yellow'])
        layout.addWidget(title)

        manager_box = QWidget()
        manager_layout = QGridLayout(manager_box)
        self.connection = self._state_label('DISCONNECTED')
        self.core_state = self._state_label('UNKNOWN')
        self.manager_detail = QLabel('Robot Managerを検索しています')
        self.error_detail = QLabel('')
        manager_layout.addWidget(self._caption('JETSON / ROBOT MANAGER'), 0, 0)
        manager_layout.addWidget(self._caption('ROBOT CORE'), 0, 1)
        manager_layout.addWidget(self.connection, 1, 0)
        manager_layout.addWidget(self.core_state, 1, 1)
        manager_layout.addWidget(self.manager_detail, 2, 0, 1, 2)
        manager_layout.addWidget(self.error_detail, 3, 0, 1, 2)
        controls = QHBoxLayout()
        self.start_button = QPushButton('▶  START ROBOT CORE')
        self.stop_button = QPushButton('■  STOP ROBOT CORE')
        self.error_button = QPushButton('!  MOCK ERROR')
        self.error_button.clicked.connect(self.backend.request_mock_error)
        self.error_button.setVisible(self.backend.backend_mode == 'mock')
        self.start_button.clicked.connect(self._start)
        self.stop_button.clicked.connect(self._stop)
        controls.addWidget(self.start_button)
        controls.addWidget(self.stop_button)
        controls.addWidget(self.error_button)
        controls.addStretch(1)
        manager_layout.addLayout(controls, 4, 0, 1, 2)
        layout.addWidget(manager_box)

        self.tabs = QTabWidget()
        self.telemetry_panel = TelemetryPanel()
        self.camera_panel = CameraPanel(backend, config)
        self.localization_panel = LocalizationPanel()
        self.recording_panel = RecordingPanel(backend, config)
        self.tabs.addTab(self.telemetry_panel, 'STATUS')
        self.tabs.addTab(self.camera_panel, 'OAK-D CAMERA')
        self.tabs.addTab(self.localization_panel, 'LOCALIZATION / GNSS')
        self.tabs.addTab(self.recording_panel, 'ROS BAG')
        layout.addWidget(self.tabs, 1)
        self.setCentralWidget(root)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(int(1000.0 / max(1.0, float(ui_config.get('refresh_hz', 4.0)))))

    @staticmethod
    def _caption(text):
        label = QLabel(text)
        label.setStyleSheet('color:' + PALETTE['blue'] + '; font-size:12px; font-weight:bold;')
        return label

    @staticmethod
    def _state_label(text):
        label = QLabel(text)
        label.setStyleSheet('background:' + PALETTE['panel'] + '; padding:8px; font-size:20px; font-weight:bold;')
        return label

    def _start(self):
        self.backend.request_start()

    def _stop(self):
        answer = QMessageBox.warning(
            self, 'Robot Coreを停止',
            'Robot Coreのlaunch全体を停止します。記録中のGUI rosbagは先に正常終了します。実行しますか？',
            QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Cancel)
        if answer == QMessageBox.Yes:
            self.backend.request_stop()

    def _refresh(self):
        status = self.backend.snapshot()
        connection, state = status['connection'], status['core_state']
        colors = {'CONNECTED': PALETTE['green'], 'STALE': PALETTE['yellow'],
                  'DISCONNECTED': PALETTE['red']}
        state_colors = {'STOPPED': PALETTE['muted'], 'STARTING': PALETTE['yellow'],
                        'RUNNING': PALETTE['green'], 'STOPPING': PALETTE['yellow'],
                        'ERROR': PALETTE['red']}
        values = ((self.connection, connection, colors.get(connection, PALETTE['red'])),
                  (self.core_state, state, state_colors.get(state, PALETTE['muted'])))
        for widget, value, color in values:
            widget.setText(value)
            widget.setStyleSheet('background:' + PALETTE['panel'] + '; color:' + color
                                 + '; padding:8px; font-size:20px; font-weight:bold;')
        heartbeat = status['heartbeat_age_sec']
        detail = ('manager heartbeat: 待受中' if heartbeat is None else
                  'manager heartbeat: {:.1f}秒前'.format(heartbeat))
        if status['unit']:
            detail += '  /  unit: ' + status['unit']
        if status['last_response']:
            detail += '  /  ' + status['last_response']
        self.manager_detail.setText(detail)
        self.error_detail.setText(status['error'])
        self.error_detail.setStyleSheet('color:' + PALETTE['red'] + '; font-weight:bold;')
        connected = connection == 'CONNECTED'
        self.start_button.setEnabled(connected and state in ('STOPPED', 'ERROR'))
        self.stop_button.setEnabled(connected and state in ('RUNNING', 'ERROR'))
        self.telemetry_panel.refresh(status, self.config)
        self.camera_panel.refresh(status)
        self.localization_panel.refresh(status)
        self.recording_panel.refresh(status)


def load_config(path):
    with open(path, 'r', encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    return document.get('operator_console', {}).get('ros__parameters', {})


def main(args=None):
    parser = argparse.ArgumentParser(description='pm_gui operator console')
    parser.add_argument('--config', required=True)
    parsed, ros_args = parser.parse_known_args(args)
    config = load_config(os.path.expanduser(parsed.config))
    rclpy.init(args=ros_args)
    backend = RosBackend(config)
    executor_thread = threading.Thread(target=rclpy.spin, args=(backend,), daemon=True)
    executor_thread.start()
    application = QApplication(sys.argv[:1])
    # Docker imageに同梱した日本語fontを選び、Qt標準fontでの豆腐表示を防ぐ。
    available_fonts = set(QFontDatabase().families())
    for family in ('Noto Sans CJK JP', 'Noto Sans JP'):
        if family in available_fonts:
            application.setFont(QFont(family, 10))
            break
    # signalからQt callbackへKeyboardInterruptを投げず、event loopを通常終了させる。
    signal.signal(signal.SIGINT, lambda _signum, _frame: application.quit())
    signal.signal(signal.SIGTERM, lambda _signum, _frame: application.quit())
    window = OperatorWindow(backend, config)
    window.show()
    try:
        exit_code = application.exec_()
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        backend.stop_recordings_for_exit()
        backend.destroy_node()
        rclpy.shutdown()
    executor_thread.join(timeout=2.0)
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
