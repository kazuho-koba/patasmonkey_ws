#!/usr/bin/env python3
"""Qt operator console。ROS executorは別threadで動作させる。"""

import argparse
import os
import signal
import sys
import threading

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QFontDatabase
from PyQt5.QtWidgets import (
    QApplication, QGridLayout, QHBoxLayout, QLabel, QMainWindow, QMessageBox,
    QPushButton, QSplitter, QVBoxLayout, QWidget,
)

from .panels import CameraPanel, LocalizationPanel, PALETTE, RecordingPanel, TelemetryPanel
from .ros_backend import RosBackend


class OperatorWindow(QMainWindow):
    """Robot Core管理と各種監視情報を一画面にまとめて表示する。"""

    def __init__(self, backend, config):
        super().__init__()
        self.backend = backend
        self.config = config
        ui_config = config.get('ui', {})
        self.setWindowTitle(ui_config.get('window_title', 'Patasmonkey UGV Operator Console'))
        requested_size = [int(value) for value in ui_config.get('window_size', [1120, 780])]
        screen = QApplication.primaryScreen()
        available_geometry = screen.availableGeometry() if screen is not None else None
        window_size = list(requested_size)
        if available_geometry is not None:
            # taskbarとwindow frameの余白を見込み、設定した初期寸法を画面内へ収める。
            window_size[0] = min(window_size[0], max(1, available_geometry.width() - 32))
            window_size[1] = min(window_size[1], max(1, available_geometry.height() - 64))
        self.resize(*window_size)
        self.setStyleSheet(
            'QMainWindow, QWidget { background:' + PALETTE['background'] + '; color:' + PALETTE['text'] + '; }'
            'QGroupBox { border:2px solid ' + PALETTE['muted'] + '; margin-top:10px; padding:8px; font-weight:bold; }'
            'QGroupBox::title { subcontrol-origin:margin; left:8px; padding:0 4px; color:' + PALETTE['blue'] + '; }'
            'QPushButton { background:' + PALETTE['panel'] + '; color:' + PALETTE['text'] + '; border:2px solid '
            + PALETTE['muted'] + '; padding:8px 14px; font-weight:bold; }'
            'QPushButton:disabled { color:' + PALETTE['muted'] + '; }'
            'QCheckBox { spacing:10px; padding:6px; }'
            'QSplitter::handle { background:' + PALETTE['muted'] + '; }'
        )
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setContentsMargins(10, 8, 10, 10)
        layout.setSpacing(6)
        title = QLabel('PATASMONKEY  /  OPERATOR CONSOLE')
        title.setFixedHeight(30)
        title.setStyleSheet(
            'background:' + PALETTE['panel'] + '; color:' + PALETTE['yellow']
            + '; font-size:16px; font-weight:bold; padding-left:8px;')
        layout.addWidget(title)

        manager_box = QWidget()
        manager_layout = QGridLayout(manager_box)
        manager_layout.setContentsMargins(4, 2, 4, 2)
        manager_layout.setHorizontalSpacing(4)
        manager_layout.setVerticalSpacing(2)
        self.connection = self._state_label('DISCONNECTED')
        self.core_state = self._state_label('UNKNOWN')
        self.manager_detail = QLabel('Robot Managerを検索しています')
        self.error_detail = QLabel('')
        self.manager_detail.setWordWrap(True)
        self.error_detail.setWordWrap(True)
        self.manager_detail.setStyleSheet('font-size:10px;')
        self.error_detail.setStyleSheet('font-size:10px; color:' + PALETTE['red'] + '; font-weight:bold;')
        self.error_detail.setVisible(False)
        manager_layout.addWidget(self._caption('JETSON / ROBOT MANAGER'), 0, 0)
        manager_layout.addWidget(self._caption('ROBOT CORE'), 0, 1)
        manager_layout.addWidget(self.connection, 1, 0)
        manager_layout.addWidget(self.core_state, 1, 1)
        manager_layout.addWidget(self.manager_detail, 2, 0, 1, 2)
        manager_layout.addWidget(self.error_detail, 3, 0, 1, 2)
        controls = QHBoxLayout()
        self.core_action_button = QPushButton('START ROBOT CORE')
        self.core_action_button.setMinimumHeight(54)
        self.core_action_button.setMinimumWidth(280)
        self.core_action_button.setEnabled(False)
        self.core_action_button.setStyleSheet(
            'background:' + PALETTE['panel'] + '; color:' + PALETTE['text']
            + '; border:1px solid ' + PALETTE['muted']
            + '; padding:8px 12px; font-size:16px; font-weight:bold;')
        self._core_action = None
        self.error_button = QPushButton('!  MOCK ERROR')
        self.error_button.clicked.connect(self.backend.request_mock_error)
        self.error_button.setVisible(self.backend.backend_mode == 'mock')
        self.core_action_button.clicked.connect(self._toggle_core_action)
        self.error_button.setStyleSheet(
            'background:' + PALETTE['panel'] + '; color:' + PALETTE['text']
            + '; border:1px solid ' + PALETTE['muted']
            + '; padding:3px 7px; font-size:10px;')
        controls.addWidget(self.core_action_button, 1)
        controls.addWidget(self.error_button)
        manager_layout.addLayout(controls, 4, 0, 1, 2)

        # 左側はcameraを上段、姿勢とjoystickを下段に置く。
        self.telemetry_panel = TelemetryPanel(config)
        self.camera_panel = CameraPanel(backend, config)
        self.localization_panel = LocalizationPanel(config)
        self.recording_panel = RecordingPanel(backend, config)

        left_column = QSplitter(Qt.Vertical)
        left_column.setChildrenCollapsible(False)
        left_column.setHandleWidth(6)
        left_column.addWidget(self.camera_panel)
        left_column.addWidget(self.telemetry_panel)
        left_column.setStretchFactor(0, 1)
        left_column.setStretchFactor(1, 1)
        left_column.setSizes([3, 1])

        # 画面タイトルはdashboardの外に置き、右下にはmanagerとbag操作だけを詰めて表示する。
        right_utility = QWidget()
        right_layout = QVBoxLayout(right_utility)
        right_layout.setContentsMargins(2, 2, 2, 2)
        right_layout.setSpacing(3)
        right_layout.addWidget(manager_box)
        right_layout.addWidget(self.recording_panel)

        right_panels = QSplitter(Qt.Vertical)
        right_panels.setChildrenCollapsible(False)
        right_panels.setHandleWidth(6)
        right_panels.addWidget(self.localization_panel)
        right_panels.addWidget(right_utility)
        right_panels.setStretchFactor(0, 1)
        right_panels.setStretchFactor(1, 1)
        right_panels.setSizes([2, 1])

        dashboard = QSplitter(Qt.Horizontal)
        dashboard.setChildrenCollapsible(False)
        dashboard.setHandleWidth(8)
        dashboard.addWidget(left_column)
        dashboard.addWidget(right_panels)
        dashboard.setStretchFactor(0, 1)
        dashboard.setStretchFactor(1, 1)
        dashboard.setSizes([3, 2])
        layout.addWidget(dashboard, 1)
        self.setCentralWidget(root)
        if available_geometry is not None:
            # 初回表示をprimary monitorの作業領域中央に置き、別画面へ移動・最大化も可能にする。
            self.move(
                available_geometry.x() + (available_geometry.width() - window_size[0]) // 2,
                available_geometry.y() + (available_geometry.height() - window_size[1]) // 2,
            )
        self._dashboard_splitter = dashboard
        self._left_splitter = left_column
        self._right_splitter = right_panels
        self._telemetry_splitter = self.telemetry_panel.splitter
        self._initial_split_sizes_applied = False
        if bool(ui_config.get('fixed_window_size', False)):
            # 固定が明示された設定だけ従来のwindow lockを適用する。
            self.setFixedSize(*window_size)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(int(1000.0 / max(1.0, float(ui_config.get('refresh_hz', 4.0)))))

    def showEvent(self, event):
        super().showEvent(event)
        if not self._initial_split_sizes_applied:
            self._initial_split_sizes_applied = True
            # 初回表示後に実寸を使って比率を適用する。camera画像はpanel内で元のaspect ratioを保つ。
            QTimer.singleShot(0, self._apply_initial_split_sizes)

    def _apply_initial_split_sizes(self):
        self._set_split_ratio(self._dashboard_splitter, (3, 2))
        self._set_split_ratio(self._left_splitter, (3, 1))
        self._set_split_ratio(self._telemetry_splitter, (1, 1))
        self._set_split_ratio(self._right_splitter, (2, 1))

    @staticmethod
    def _set_split_ratio(splitter, ratio):
        available = splitter.width() - splitter.handleWidth() * (len(ratio) - 1)
        if available <= 0:
            return
        total = float(sum(ratio))
        splitter.setSizes([max(1, int(available * part / total)) for part in ratio])

    @staticmethod
    def _caption(text):
        label = QLabel(text)
        label.setStyleSheet('color:' + PALETTE['blue'] + '; font-size:12px; font-weight:bold;')
        return label

    @staticmethod
    def _state_label(text):
        label = QLabel(text)
        label.setStyleSheet('background:' + PALETTE['panel'] + '; padding:4px; font-size:14px; font-weight:bold;')
        return label

    def _start(self):
        self.backend.request_start()

    def _toggle_core_action(self):
        if self._core_action == 'start':
            self._start()
        elif self._core_action == 'stop':
            self._stop()

    def _stop(self):
        answer = QMessageBox.warning(
            self, 'Robot Coreを停止',
            'Jetson上のMission/Debug bagへ停止を依頼し、metadataとros2 bag infoの保存検証が完了してからCoreを停止します。検証に失敗した場合はCore停止を中断します。実行しますか？',
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
                                 + '; padding:4px; font-size:14px; font-weight:bold;')
        heartbeat = status['heartbeat_age_sec']
        detail = ('manager heartbeat: 待受中' if heartbeat is None else
                  'manager heartbeat: {:.1f}秒前'.format(heartbeat))
        if status['unit']:
            detail += '  /  unit: ' + status['unit']
        if status['last_response']:
            detail += '  /  ' + status['last_response']
        self.manager_detail.setText(detail)
        self.error_detail.setText(status['error'])
        self.error_detail.setVisible(bool(status['error']))
        connected = connection == 'CONNECTED'
        self._update_core_action(connected, state)
        self.telemetry_panel.refresh(status, self.config)
        self.camera_panel.refresh(status)
        self.localization_panel.refresh(status)
        self.recording_panel.refresh(status)

    def _update_core_action(self, connected, state):
        """Robot Core状態に応じて単一ボタンの動作・表示・安全状態を切り替える。"""
        if state == 'STOPPED':
            text, action, background = '▶  START ROBOT CORE', 'start', PALETTE['green']
        elif state == 'RUNNING':
            text, action, background = '■  STOP ROBOT CORE', 'stop', PALETTE['red']
        elif state == 'ERROR':
            text, action, background = '↻  RETRY START ROBOT CORE', 'start', PALETTE['yellow']
        elif state in ('STARTING', 'STOPPING'):
            verb = 'STARTING' if state == 'STARTING' else 'STOPPING'
            text, action, background = '…  {} ROBOT CORE'.format(verb), None, PALETTE['yellow']
        else:
            text, action, background = 'ROBOT CORE STATUS UNKNOWN', None, PALETTE['panel']
        self._core_action = action
        self.core_action_button.setText(text)
        self.core_action_button.setEnabled(connected and action is not None)
        foreground = (PALETTE['background'] if state in
                      ('STOPPED', 'RUNNING', 'ERROR', 'STARTING', 'STOPPING')
                      else PALETTE['text'])
        self.core_action_button.setStyleSheet(
            'background:' + background + '; color:' + foreground
            + '; border:2px solid ' + PALETTE['muted']
            + '; padding:8px 12px; font-size:16px; font-weight:bold;')


def load_config(path):
    """GUI設定に、joy_teleopが実際に使う軸・ボタン割当を追加する。"""
    with open(path, 'r', encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    config = document.get('operator_console', {}).get('ros__parameters', {})
    joystick = config.setdefault('joystick', {})
    package_name = joystick.get('config_package', 'pm_teleop')
    teleop_file = joystick.get('teleop_config', 'config/teleop_twist_joy.yaml')
    joy_file = joystick.get('joy_config', 'config/joy_params.yaml')
    try:
        package_share = get_package_share_directory(package_name)
        teleop_path = teleop_file if os.path.isabs(teleop_file) else os.path.join(
            package_share, teleop_file)
        joy_path = joy_file if os.path.isabs(joy_file) else os.path.join(
            package_share, joy_file)
        with open(teleop_path, 'r', encoding='utf-8') as stream:
            teleop_document = yaml.safe_load(stream) or {}
        with open(joy_path, 'r', encoding='utf-8') as stream:
            joy_document = yaml.safe_load(stream) or {}
        teleop = teleop_document['joy_teleop']['ros__parameters']
        joy_node = joy_document['joy_node']['ros__parameters']
        linear_axes = teleop.get('axis_linear', {})
        angular_axes = teleop.get('axis_angular', {})
        # teleop_twist_joyが読む同一YAMLから、Joy.axes/buttonsの0始まりindexを取得する。
        joystick.update({
            'axis_linear_x': int(linear_axes.get('x', -1)),
            'axis_angular_yaw': int(angular_axes.get('yaw', -1)),
            'enable_button': int(teleop.get('enable_button', -1)),
            'enable_turbo_button': int(teleop.get('enable_turbo_button', -1)),
            'require_enable_button': bool(teleop.get('require_enable_button', False)),
            'deadzone': float(joy_node.get('deadzone', 0.0)),
            'config_source': '{} / {}'.format(teleop_path, joy_path),
            'config_error': '',
        })
    except Exception as exc:
        # joystick設定が読めない場合もGUI全体は起動し、monitorには設定エラーを表示する。
        joystick['config_error'] = '{}: {}'.format(package_name, exc)
        print('[WARN] joystick設定を読み込めません: {}'.format(exc), file=sys.stderr)
    return config


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
