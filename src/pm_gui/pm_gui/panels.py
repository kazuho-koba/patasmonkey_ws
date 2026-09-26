"""センサー、localization、rosbag表示用のQt widget群。"""

import math

from PyQt5.QtCore import Qt, QRectF
from PyQt5.QtGui import QColor, QImage, QPainter, QPen, QPolygonF, QPixmap
from PyQt5.QtWidgets import (
    QCheckBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton,
    QVBoxLayout, QWidget,
)


PALETTE = {
    'background': '#101820', 'panel': '#1d2b36', 'text': '#f0f3bd',
    'green': '#7ac943', 'yellow': '#f4d35e', 'red': '#ee6352',
    'blue': '#4ea5d9', 'muted': '#91a6b2',
}


class AttitudeCanvas(QWidget):
    """roll/pitchを反映した簡易UGV模式図。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll = 0.0
        self.pitch = 0.0
        self.setMinimumSize(220, 170)

    def set_attitude(self, roll, pitch):
        self.roll, self.pitch = roll, pitch
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, False)
        painter.fillRect(self.rect(), QColor(PALETTE['panel']))
        cx, cy = self.width() / 2.0, self.height() / 2.0
        # UGV deckをrollで回転し、pitchを画面上の前後傾きとして模式化する。
        painter.save()
        painter.translate(cx, cy + max(-25.0, min(25.0, self.pitch * 22.0)))
        painter.rotate(-math.degrees(self.roll))
        painter.setPen(QPen(QColor(PALETTE['text']), 3))
        painter.setBrush(QColor('#344b5b'))
        painter.drawRect(QRectF(-65, -31, 130, 62))
        painter.setBrush(QColor(PALETTE['blue']))
        painter.drawRect(QRectF(22, -20, 36, 40))
        painter.setBrush(QColor(PALETTE['muted']))
        for x in (-48, 38):
            for y in (-39, 27):
                painter.drawRect(QRectF(x, y, 18, 12))
        painter.setPen(QPen(QColor(PALETTE['yellow']), 4))
        painter.drawLine(-65, 0, -82, 0)
        painter.drawLine(-82, 0, -72, -8)
        painter.drawLine(-82, 0, -72, 8)
        painter.restore()


class TrajectoryCanvas(QWidget):
    """map座標の軌跡を描き、ENU位置をlat/lonとして扱わない。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.points = []
        self.heading = 0.0
        self.fix_state = 'NO FIX'
        self.setMinimumSize(360, 280)

    def add_pose(self, x, y, yaw, fix_state):
        if not self.points or math.hypot(x - self.points[-1][0], y - self.points[-1][1]) > 0.05:
            self.points.append((x, y))
            self.points = self.points[-1500:]
        self.heading, self.fix_state = yaw, fix_state
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(PALETTE['panel']))
        painter.setPen(QPen(QColor('#344b5b'), 1, Qt.DotLine))
        for x in range(20, self.width(), 40):
            painter.drawLine(x, 18, x, self.height() - 22)
        for y in range(18, self.height() - 15, 40):
            painter.drawLine(18, y, self.width() - 18, y)
        painter.setPen(QPen(QColor(PALETTE['muted']), 1))
        painter.drawLine(18, self.height() - 22, self.width() - 18, self.height() - 22)
        painter.drawLine(18, 18, 18, self.height() - 22)
        if not self.points:
            painter.setPen(QColor(PALETTE['text']))
            painter.drawText(self.rect(), Qt.AlignCenter, 'odometry待受中\naxes: map x / map y')
            return
        xs, ys = [p[0] for p in self.points], [p[1] for p in self.points]
        min_x, max_x, min_y, max_y = min(xs), max(xs), min(ys), max(ys)
        scale = min((self.width() - 52) / max(max_x - min_x, 1.0),
                    (self.height() - 56) / max(max_y - min_y, 1.0))
        def project(point):
            return (26 + (point[0] - min_x) * scale,
                    self.height() - 30 - (point[1] - min_y) * scale)
        if len(self.points) > 1:
            painter.setPen(QPen(QColor(PALETTE['blue']), 2))
            projected = [project(p) for p in self.points]
            for first, second in zip(projected[:-1], projected[1:]):
                painter.drawLine(int(first[0]), int(first[1]), int(second[0]), int(second[1]))
        px, py = project(self.points[-1])
        colors = {'NO FIX': PALETTE['red'], 'GNSS': PALETTE['yellow'],
                  'RTK FLOAT': '#ff9f1c', 'RTK FIX': PALETTE['green']}
        painter.save()
        painter.translate(px, py)
        painter.rotate(-math.degrees(self.heading))
        painter.setPen(QPen(QColor(PALETTE['text']), 2))
        painter.setBrush(QColor(colors.get(self.fix_state, PALETTE['muted'])))
        painter.drawPolygon(QPolygonF([(-11, 9), (12, 0), (-11, -9)]))
        painter.restore()
        painter.setPen(QColor(PALETTE['text']))
        painter.drawText(8, 15, 'map x / map y  (m)')


class CameraPanel(QWidget):
    def __init__(self, backend, config):
        super().__init__()
        self.backend = backend
        self.config = config.get('camera', {})
        self._last_rendered_count = 0
        self._qt_render_count = 0
        layout = QVBoxLayout(self)
        self.toggle = QCheckBox('Camera display ON')
        self.toggle.toggled.connect(backend.set_camera_enabled)
        layout.addWidget(self.toggle)

        # 状態をimage labelの下に置かず、画像より上へ固定して常に見えるようにする。
        self.state = QLabel('OFF  /  カメラ表示OFF')
        self.state.setStyleSheet(
            'background:#344b5b; color:' + PALETTE['text']
            + '; padding:8px; font-size:16px; font-weight:bold;')
        layout.addWidget(self.state)
        self.details = QLabel('')
        self.details.setWordWrap(True)
        self.details.setMinimumHeight(42)
        layout.addWidget(self.details)

        self.image = QLabel('カメラ表示をONにすると画像を受信します')
        self.image.setAlignment(Qt.AlignCenter)
        self.image.setMinimumSize(640, 320)
        self.image.setStyleSheet('background:#080d12; color:' + PALETTE['muted'] + ';')
        layout.addWidget(self.image, 1)

    def refresh(self, snapshot):
        camera = snapshot['camera']
        topic = camera['topic']
        age = camera['age']
        timeout = float(self.config.get('stale_timeout_sec', 2.0))
        if not camera['enabled']:
            state, color = 'OFF  /  カメラ表示OFF', PALETTE['muted']
        elif not camera['subscribed']:
            state, color = '購読中  /  subscriptionを作成しています', PALETTE['yellow']
        elif camera['error']:
            state, color = '変換エラー  /  cv_bridge RGB変換に失敗', PALETTE['red']
        elif camera['image'] is None:
            waiting_age = camera['subscription_age']
            if waiting_age is not None and waiting_age > timeout:
                state, color = 'STALE  /  subscription後も画像を受信していません', PALETTE['red']
            else:
                state, color = '画像待受  /  subscription接続後、最初の画像を待っています', PALETTE['yellow']
        elif age > timeout:
            state, color = 'STALE  /  最終画像がtimeoutを超えています', PALETTE['red']
        else:
            state, color = '受信中  /  RGB画像を表示しています', PALETTE['green']
        self.state.setText(state)
        self.state.setStyleSheet(
            'background:#344b5b; color:' + color
            + '; padding:8px; font-size:16px; font-weight:bold;')
        age_text = '未受信' if age is None else '{:.2f}秒前'.format(age)
        self.details.setText(
            'Topic: {}    最終受信: {}    callback: {}    cv_bridge成功: {}    変換エラー: {}    Qt描画更新: {}'.format(
                topic, age_text, camera['receive_count'], camera['converted_count'],
                camera['conversion_error_count'], self._qt_render_count))
        if camera['error']:
            self.details.setText(self.details.text() + '\nエラー: ' + camera['error'])

        frame = camera['image']
        if frame is None:
            if not camera['enabled']:
                self.image.setPixmap(QPixmap())
                self.image.setText('カメラ表示はOFFです')
            elif camera['error']:
                self.image.setText('画像の変換に失敗しました')
            else:
                self.image.setText('画像待受中')
            return
        if camera['converted_count'] == self._last_rendered_count:
            return
        height, width, channels = frame.shape
        qimage = QImage(frame.data, width, height, channels * width, QImage.Format_RGB888).copy()
        pixmap = QPixmap.fromImage(qimage).scaled(
            self.image.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.image.setPixmap(pixmap)
        self.image.setText('')
        self._last_rendered_count = camera['converted_count']
        self._qt_render_count += 1
        self.details.setText(
            'Topic: {}    最終受信: {}    callback: {}    cv_bridge成功: {}    変換エラー: {}    Qt描画更新: {}'.format(
                topic, age_text, camera['receive_count'], camera['converted_count'],
                camera['conversion_error_count'], self._qt_render_count))


class TelemetryPanel(QWidget):
    def __init__(self):
        super().__init__()
        root = QHBoxLayout(self)
        attitude = QGroupBox('ATTITUDE')
        left = QVBoxLayout(attitude)
        self.canvas = AttitudeCanvas()
        left.addWidget(self.canvas)
        self.attitude = QLabel('roll --   pitch --\nyaw --')
        left.addWidget(self.attitude)
        joystick = QGroupBox('JOYSTICK / TELEOP MONITOR')
        right = QGridLayout(joystick)
        self.mode = QLabel('NO DATA')
        self.linear = QLabel('Linear X: -- m/s')
        self.angular = QLabel('Angular Z: -- rad/s')
        right.addWidget(QLabel('Mode'), 0, 0)
        right.addWidget(self.mode, 0, 1)
        right.addWidget(self.linear, 1, 0, 1, 2)
        right.addWidget(self.angular, 2, 0, 1, 2)
        self.note = QLabel('teleop経路にはpublishしません')
        self.note.setStyleSheet('color:' + PALETTE['muted'])
        right.addWidget(self.note, 3, 0, 1, 2)
        root.addWidget(attitude)
        root.addWidget(joystick, 1)

    @staticmethod
    def _fresh(snapshot, key):
        item = snapshot['telemetry'].get(key)
        return item['value'] if item and item['fresh'] else None

    def refresh(self, snapshot, config):
        odom = self._fresh(snapshot, 'odometry')
        if odom:
            roll, pitch, yaw = odom['roll'], odom['pitch'], odom['yaw']
            bearing = (90.0 - math.degrees(yaw)) % 360.0
            cardinal = ('N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE',
                        'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW')
            direction = cardinal[int((bearing + 11.25) // 22.5) % 16]
            self.attitude.setText('Roll: {:+.1f}°   Pitch: {:+.1f}°\nYaw: {:.1f}° / {}'.format(
                math.degrees(roll), math.degrees(pitch), bearing, direction))
            self.canvas.set_attitude(roll, pitch)
        else:
            self.attitude.setText('姿勢データ stale / 待受中')
        joy = self._fresh(snapshot, 'joy')
        twist = self._fresh(snapshot, 'twist')
        joy_cfg = config.get('joystick', {})
        if joy is None:
            mode = 'STALE / NO DATA'
        else:
            buttons = joy['buttons']
            enable_i = int(joy_cfg.get('enable_button', 4))
            turbo_i = int(joy_cfg.get('turbo_button', 1))
            enabled = enable_i < len(buttons) and buttons[enable_i] != 0
            turbo = turbo_i < len(buttons) and buttons[turbo_i] != 0
            mode = 'TURBO' if enabled and turbo else 'ENABLED' if enabled else 'DISABLED'
        colors = {'DISABLED': PALETTE['muted'], 'ENABLED': PALETTE['green'],
                  'TURBO': PALETTE['yellow']}
        self.mode.setText(mode)
        self.mode.setStyleSheet('font-weight:bold; color:' + colors.get(mode, PALETTE['red']))
        if twist:
            self.linear.setText('Linear X: {:+.3f} m/s'.format(twist['linear_x']))
            self.angular.setText('Angular Z: {:+.3f} rad/s'.format(twist['angular_z']))
        else:
            self.linear.setText('Linear X: stale / -- m/s')
            self.angular.setText('Angular Z: stale / -- rad/s')


class LocalizationPanel(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        self.map = TrajectoryCanvas()
        layout.addWidget(self.map, 1)
        self.position = QLabel('位置: 待受中')
        self.gnss = QLabel('GNSS: NO FIX')
        layout.addWidget(self.position)
        layout.addWidget(self.gnss)

    def refresh(self, snapshot):
        telemetry = snapshot['telemetry']
        odom_item = telemetry.get('odometry')
        navpvt_item = telemetry.get('navpvt')
        if navpvt_item and navpvt_item['fresh']:
            fix_item = navpvt_item
        else:
            fix_item = telemetry.get('fix')
        fix = fix_item['value'] if fix_item and fix_item['fresh'] else None
        if fix is None:
            state = 'STALE / NO DATA'
        elif 'gnss_state' in fix:
            state = fix['gnss_state']
        else:
            state = 'GNSS' if fix['status'] >= 0 else 'NO FIX'
        self.gnss.setText('GNSS: ' + state)
        color = {'NO FIX': PALETTE['red'], 'GNSS': PALETTE['yellow'],
                 'RTK FLOAT': '#ff9f1c', 'RTK FIX': PALETTE['green'],
                 'STALE / NO DATA': PALETTE['muted']}.get(state, PALETTE['muted'])
        self.gnss.setStyleSheet('font-weight:bold; color:' + color)
        if odom_item and odom_item['fresh']:
            odom = odom_item['value']
            self.map.add_pose(odom['x'], odom['y'], odom['yaw'], state)
            self.position.setText('map ({})  x: {:.2f} m  y: {:.2f} m  z: {:.2f} m'.format(
                odom['frame_id'] or 'unknown frame', odom['x'], odom['y'], odom['z']))
        else:
            self.position.setText('odometry stale / 待受中')
        fix_value = fix if fix and 'latitude' in fix else None
        if fix_value:
            self.position.setText(self.position.text() +
                '    WGS84: {:.7f}, {:.7f}'.format(fix_value['latitude'], fix_value['longitude']))


class RecordingPanel(QWidget):
    def __init__(self, backend, config):
        super().__init__()
        self.backend = backend
        self.config = config.get('recording', {})
        layout = QVBoxLayout(self)
        self.directory = QLabel('')
        self.disk = QLabel('')
        layout.addWidget(self.directory)
        layout.addWidget(self.disk)
        self.external = QLabel('')
        layout.addWidget(self.external)
        self.rows = {}
        profiles = self.config.get('profiles', {})
        for profile in ('mission', 'debug'):
            row = QHBoxLayout()
            name = QLabel('{} recording'.format(profile.upper()))
            state = QLabel('STOPPED')
            detail = QLabel('')
            start = QPushButton('START')
            stop = QPushButton('STOP')
            start.clicked.connect(lambda _checked=False, p=profile: self._start(p))
            stop.clicked.connect(lambda _checked=False, p=profile: self.backend.stop_recording(p))
            row.addWidget(name)
            row.addWidget(state)
            row.addWidget(detail, 1)
            row.addWidget(start)
            row.addWidget(stop)
            layout.addLayout(row)
            self.rows[profile] = (state, detail, start, stop, profiles.get(profile, {}))
        self.warning = QLabel('')
        self.warning.setStyleSheet('font-weight:bold; color:' + PALETTE['red'])
        layout.addWidget(self.warning)
        layout.addStretch(1)

    def _start(self, profile):
        ok, message = self.backend.start_recording(profile)
        if not ok:
            self.warning.setText(message)

    def refresh(self, snapshot):
        self.directory.setText('Output: ' + snapshot['recording_directory'])
        free = snapshot['free_bytes']
        if free is None:
            self.disk.setText('Disk free: 不明')
            low = False
        else:
            free_gib = free / float(1024 ** 3)
            self.disk.setText('Disk free: {:.1f} GiB'.format(free_gib))
            low = free < int(float(self.config.get('low_disk_gib', 5.0)) * 1024 ** 3)
        self.warning.setText('空き容量が設定値を下回っています' if low else '')
        self.external.setText('Robot Core側 rosbag recorder: 検出' if snapshot['external_recording']
                              else 'Robot Core側 rosbag recorder: 未検出')
        for profile, (state, detail, start, stop, _settings) in self.rows.items():
            item = snapshot['recordings'].get(profile)
            current = item['state'] if item else 'STOPPED'
            state.setText(current)
            state.setStyleSheet('font-weight:bold; color:' +
                                (PALETTE['red'] if current == 'RECORDING' else PALETTE['muted']))
            if item:
                elapsed = max(0, int(__import__('time').monotonic() - item['started']))
                detail.setText('{}  /  {:02d}:{:02d}'.format(item['output'], elapsed // 60, elapsed % 60))
            else:
                detail.setText('')
            start.setEnabled(current == 'STOPPED')
            stop.setEnabled(current == 'RECORDING')
