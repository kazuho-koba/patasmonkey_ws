"""センサー、localization、rosbag表示用のQt widget群。"""

import math

import numpy as np
from PyQt5.QtCore import QPointF, Qt, QRectF, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen, QPolygonF, QPixmap
from PyQt5.QtWidgets import (
    QCheckBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton,
    QSplitter, QSizePolicy, QVBoxLayout, QWidget, QMessageBox,
)

from .map_tiles import MapTileProvider
from .urdf_model import UrdfRobotModel


PALETTE = {
    'background': '#101820', 'panel': '#1d2b36', 'text': '#f0f3bd',
    'green': '#7ac943', 'yellow': '#f4d35e', 'red': '#ee6352',
    'blue': '#4ea5d9', 'muted': '#91a6b2',
}


class AttitudeCanvas(QWidget):
    """pm_descriptionのURDF meshへROS roll/pitch/yawを適用して描画する。"""

    def __init__(self, config=None, parent=None):
        super().__init__(parent)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        attitude_config = config or {}
        # 描画倍率はwidgetのresizeや姿勢角では変えず、YAMLのpixel/mを一定に使う。
        self.pixels_per_meter = max(
            20.0, float(attitude_config.get('pixels_per_meter', 240.0)))
        try:
            self.model = UrdfRobotModel(attitude_config.get('urdf_path'))
            self.model_error = ''
        except Exception as exc:
            self.model = None
            self.model_error = str(exc)
        self.setMinimumSize(180, 135)

    def set_attitude(self, roll, pitch, yaw):
        self.roll, self.pitch, self.yaw = roll, pitch, yaw
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(PALETTE['panel']))
        if self.model is None:
            painter.setPen(QColor(PALETTE['red']))
            painter.drawText(self.rect(), Qt.AlignCenter,
                             'URDF modelを読み込めません\n' + self.model_error[:120])
            return

        # ROS姿勢をRz(yaw) * Ry(pitch) * Rx(roll)でbase_linkの+X/+Y/+ZからENUへ回す。
        cr, sr = math.cos(self.roll), math.sin(self.roll)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        rotation = np.asarray((
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        ), dtype=np.float64)
        triangles = self.model.triangles @ rotation.T
        normals = self.model.normals @ rotation.T

        # East(+X)を画面右へ固定し、North(+Y)とUp(+Z)を斜め上から見せる。
        elevation = math.radians(32.0)
        projected_up = (triangles[:, :, 1] * math.sin(elevation)
                        + triangles[:, :, 2] * math.cos(elevation))
        depth = (triangles[:, :, 1] * math.cos(elevation)
                 - triangles[:, :, 2] * math.sin(elevation))

        # 車体の大きさから一定の平面範囲を作り、水平面をbody中心の高さに置く。
        neutral_vertices = self.model.triangles.reshape((-1, 3))
        half_x = float(np.max(np.abs(neutral_vertices[:, 0]))) + 0.035
        half_y = float(np.max(np.abs(neutral_vertices[:, 1]))) + 0.035
        body_center = self.model.body_center_anchor @ rotation.T
        # 水平面はyawだけ車体に追従し、roll/pitchには追従しない。
        yaw_rotation = np.asarray(((cy, -sy), (sy, cy)), dtype=np.float64)
        # 投影中心はbody中心に固定し、widget幅・高さや姿勢に応じた自動fitをしない。
        scale = self.pixels_per_meter
        center_x = float(body_center[0])
        center_y = float(body_center[1] * math.sin(elevation)
                         + body_center[2] * math.cos(elevation))

        # 小さなURDF meshはflat shadingで描き、奥行き順に重ねて軽量表示する。
        light = np.asarray((0.25, -0.45, 0.86), dtype=np.float64)
        light /= np.linalg.norm(light)
        illumination = 0.32 + 0.68 * np.maximum(0.0, normals @ light)
        painter.save()
        painter.translate(self.width() * 0.5, self.height() * 0.5)

        def project_world(point):
            up = point[1] * math.sin(elevation) + point[2] * math.cos(elevation)
            return QPointF((point[0] - center_x) * scale,
                           -(up - center_y) * scale)

        def camera_depth(point):
            # 南側・上方に置いた固定cameraからの奥行き座標。
            return point[1] * math.cos(elevation) - point[2] * math.sin(elevation)

        # 面を格子cellへ分割し、body facetと同じ奥行き順へ入れて部分的な遮蔽を描く。
        grid_step = 0.10

        def grid_levels(half_extent):
            values = list(np.arange(-half_extent, half_extent, grid_step))
            values.append(half_extent)
            return values

        grid_x, grid_y = grid_levels(half_x), grid_levels(half_y)

        def plane_world_point(x, y):
            xy = np.asarray((x, y)) @ yaw_rotation.T + body_center[:2]
            return np.asarray((xy[0], xy[1], body_center[2]), dtype=np.float64)

        drawables = []
        for index in range(len(triangles)):
            drawables.append((float(depth[index].mean()), 1, 'mesh', index))

        plane_grid_color = QColor(145, 215, 242, 145)
        plane_fill_color = QColor(55, 145, 190, 48)
        plane_pen = QPen(plane_grid_color, 1, Qt.DotLine)
        for x_index in range(len(grid_x) - 1):
            x0, x1 = grid_x[x_index], grid_x[x_index + 1]
            for y_index in range(len(grid_y) - 1):
                y0, y1 = grid_y[y_index], grid_y[y_index + 1]
                cell = [plane_world_point(x0, y0), plane_world_point(x1, y0),
                        plane_world_point(x1, y1), plane_world_point(x0, y1)]
                for face in ((0, 1, 2), (0, 2, 3)):
                    corners = [cell[item] for item in face]
                    mean_depth = sum(camera_depth(point) for point in corners) / 3.0
                    drawables.append((mean_depth, 0, 'plane',
                                      [project_world(point) for point in corners]))

        # 格子線も細かいsegmentに分けてbodyとの前後関係を保つ。
        grid_segments = []
        for x in grid_x:
            for y0, y1 in zip(grid_y[:-1], grid_y[1:]):
                grid_segments.append((plane_world_point(x, y0), plane_world_point(x, y1)))
        for y in grid_y:
            for x0, x1 in zip(grid_x[:-1], grid_x[1:]):
                grid_segments.append((plane_world_point(x0, y), plane_world_point(x1, y)))
        for first, second in grid_segments:
            mean_depth = (camera_depth(first) + camera_depth(second)) * 0.5
            drawables.append((mean_depth, 0, 'grid',
                              (project_world(first), project_world(second))))

        # 遠い面から近いmeshへ描画する。手前のbody triangleが水平面・格子を隠す。
        for _depth, _priority, kind, item in sorted(
                drawables, key=lambda drawable: (-drawable[0], drawable[1])):
            if kind == 'mesh':
                points = triangles[item]
                polygon = QPolygonF([
                    QPointF((point[0] - center_x) * scale,
                            -(up - center_y) * scale)
                    for point, up in zip(points, projected_up[item])
                ])
                base = self.model.colors[item]
                shade = float(illumination[item])
                painter.setPen(Qt.NoPen)
                painter.setBrush(QColor(
                    int(base[0] * shade), int(base[1] * shade), int(base[2] * shade)))
                painter.drawPolygon(polygon)
            elif kind == 'plane':
                painter.setPen(Qt.NoPen)
                painter.setBrush(plane_fill_color)
                painter.drawPolygon(QPolygonF(item))
            else:
                painter.setPen(plane_pen)
                painter.setBrush(Qt.NoBrush)
                painter.drawLine(item[0], item[1])

        # URDF body meshから求めた前端中央を姿勢回転し、そこを矢印の根元にする。
        forward = np.asarray((rotation[0, 0],
                              -(rotation[1, 0] * math.sin(elevation)
                                + rotation[2, 0] * math.cos(elevation))))
        forward /= max(float(np.linalg.norm(forward)), 1e-9)
        body_front = self.model.body_front_anchor @ rotation.T
        anchor_up = (body_front[1] * math.sin(elevation)
                     + body_front[2] * math.cos(elevation))
        tail = np.asarray(((body_front[0] - center_x) * scale,
                           -(anchor_up - center_y) * scale))
        tip = tail + forward * 21.0
        perpendicular = np.asarray((-forward[1], forward[0]))
        arrow_head = QPolygonF([
            QPointF(tip[0], tip[1]),
            QPointF(tip[0] - forward[0] * 7.0 + perpendicular[0] * 3.5,
                    tip[1] - forward[1] * 7.0 + perpendicular[1] * 3.5),
            QPointF(tip[0] - forward[0] * 7.0 - perpendicular[0] * 3.5,
                    tip[1] - forward[1] * 7.0 - perpendicular[1] * 3.5),
        ])
        painter.setPen(QPen(QColor(PALETTE['yellow']), 3))
        painter.setBrush(QColor(PALETTE['yellow']))
        painter.drawLine(QPointF(tail[0], tail[1]), QPointF(tip[0], tip[1]))
        painter.drawPolygon(arrow_head)
        painter.restore()

        painter.setPen(QColor(PALETTE['muted']))
        painter.drawText(6, 15, 'URDF  /  矢印:前方  青:水平面')


class TrajectoryCanvas(QWidget):
    """GNSSが有効なら地理タイル、無効ならmap座標grid上に軌跡を描く。"""

    zoom_changed = pyqtSignal(int)
    follow_changed = pyqtSignal(bool)

    def __init__(self, map_config=None, parent=None):
        super().__init__(parent)
        self.map_config = map_config or {}
        self.points = []
        self.heading = 0.0
        self.fix_state = 'NO FIX'
        self.geo_center = None
        self.geo_position = None
        self.gnss_points = []
        self.geo_yaw = None
        self._follow_vehicle = True
        self._drag_last_position = None
        self._controls_overlay = None
        # tile取得上限と表示上限を分け、画像だけを拡大する。
        self.tile_max_zoom = max(1, min(30, int(self.map_config.get('tile_max_zoom', 19))))
        self.max_zoom = max(self.tile_max_zoom, min(30, int(self.map_config.get('max_zoom', 26))))
        self._zoom = max(1, min(self.max_zoom, int(self.map_config.get('zoom', 17))))
        self._tile_provider = MapTileProvider(self.map_config, self)
        self._tile_provider.tiles_changed.connect(self.update)
        self.setMinimumSize(260, 170)
        self.setMouseTracking(True)

    def set_controls_overlay(self, widget):
        """地図操作ボタンをcanvas内に配置し、地図画像へ重ねて表示する。"""
        self._controls_overlay = widget
        self._position_controls_overlay()
        widget.show()

    def _position_controls_overlay(self):
        if self._controls_overlay is None:
            return
        self._controls_overlay.adjustSize()
        x = max(6, self.width() - self._controls_overlay.width() - 6)
        self._controls_overlay.move(x, 6)
        self._controls_overlay.raise_()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._position_controls_overlay()

    @property
    def zoom(self):
        return self._zoom

    def set_zoom(self, zoom):
        """地図縮尺を設定上限まで変更する。tile取得の上限は独立して維持する。"""
        zoom = max(1, min(self.max_zoom, int(zoom)))
        if zoom == self._zoom:
            return
        self._zoom = zoom
        self.update()
        self.zoom_changed.emit(self._zoom)

    def wheelEvent(self, event):
        """地図上のホイール1回で縮尺を1段階変更する。"""
        delta = event.angleDelta().y()
        if delta == 0:
            event.ignore()
            return
        self.set_zoom(self._zoom + (1 if delta > 0 else -1))
        event.accept()

    def add_pose(self, x, y, yaw, fix_state):
        if not self.points or math.hypot(x - self.points[-1][0], y - self.points[-1][1]) > 0.05:
            self.points.append((x, y))
            self.points = self.points[-1500:]
        self.heading, self.fix_state = yaw, fix_state
        self.update()

    def set_geographic_view(self, center, path, yaw, fix_state):
        """GNSS WGS84だけをWeb Mercatorへ投影し、odom値は変換しない。"""
        if center is None:
            self.geo_center = None
            self.geo_position = None
            self.gnss_points = []
            if not self._follow_vehicle:
                self._follow_vehicle = True
            self.setCursor(Qt.ArrowCursor)
        else:
            self.geo_position = (float(center[0]), float(center[1]))
            # 通常は車両を中央に追従し、手動pan中は地図中心を維持する。
            if self._follow_vehicle or self.geo_center is None:
                self.geo_center = self.geo_position
            self.gnss_points = list(path or [])
            if self._drag_last_position is None:
                self.setCursor(Qt.OpenHandCursor)
            self.geo_yaw = yaw
        self.fix_state = fix_state
        self.follow_changed.emit(self._follow_vehicle)
        self.update()

    @staticmethod
    def _geo_from_world_pixel(x, y, zoom):
        """Web Mercator world pixelをWGS84へ戻し、地図端で座標をwrapする。"""
        world_size = float((1 << zoom) * 256)
        x %= world_size
        y = max(0.0, min(world_size, y))
        longitude = x / world_size * 360.0 - 180.0
        mercator_n = math.pi - 2.0 * math.pi * y / world_size
        latitude = math.degrees(math.atan(math.sinh(mercator_n)))
        return latitude, longitude

    def _pan_by(self, dx, dy):
        """画面上のdrag量と逆向きに地図中心を動かして、地図画像を追従させる。"""
        if self.geo_center is None or (dx == 0 and dy == 0):
            return
        center_x, center_y = self._world_pixel(
            self.geo_center[0], self.geo_center[1], self._zoom)
        self.geo_center = self._geo_from_world_pixel(
            center_x - dx, center_y - dy, self._zoom)
        if self._follow_vehicle:
            self._follow_vehicle = False
            self.follow_changed.emit(False)
        self.update()

    def follow_vehicle(self):
        """GNSS現在位置を地図中央へ戻し、自動追従を再開する。"""
        if self.geo_position is None:
            return
        self._follow_vehicle = True
        self.geo_center = self.geo_position
        self.follow_changed.emit(True)
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self.geo_center is not None:
            self._drag_last_position = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if (self._drag_last_position is not None
                and event.buttons() & Qt.LeftButton):
            position = event.pos()
            delta = position - self._drag_last_position
            self._pan_by(delta.x(), delta.y())
            self._drag_last_position = position
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and self._drag_last_position is not None:
            self._drag_last_position = None
            self.setCursor(Qt.OpenHandCursor if self.geo_center is not None else Qt.ArrowCursor)
            event.accept()
            return
        super().mouseReleaseEvent(event)

    @staticmethod
    def _world_pixel(latitude, longitude, zoom):
        """WGS84緯度経度をSlippy Map/Web Mercatorのpixelへ変換する。"""
        latitude = max(-85.05112878, min(85.05112878, float(latitude)))
        longitude = max(-180.0, min(180.0, float(longitude)))
        world_size = float((1 << zoom) * 256)
        x = (longitude + 180.0) / 360.0 * world_size
        latitude_rad = math.radians(latitude)
        mercator_y = math.log((1.0 + math.sin(latitude_rad)) /
                              (1.0 - math.sin(latitude_rad))) / (4.0 * math.pi)
        y = (0.5 - mercator_y) * world_size
        return x, y

    @staticmethod
    def _nice_scale_distance(max_distance_m):
        """指定pixel幅に収まる1/2/5系列の読みやすい距離を選ぶ。"""
        if not math.isfinite(max_distance_m) or max_distance_m <= 0.0:
            return 1.0
        magnitude = 10.0 ** math.floor(math.log10(max_distance_m))
        normalized = max_distance_m / magnitude
        multiplier = 5.0 if normalized >= 5.0 else 2.0 if normalized >= 2.0 else 1.0
        return multiplier * magnitude

    def _paint_scale_bar(self, painter, zoom):
        """中心緯度とzoomから地上距離を求め、地図画像内に目盛りを描く。"""
        latitude = max(-85.05112878, min(85.05112878, self.geo_center[0]))
        # Web Mercatorのpixel ground resolutionは赤道上の値にcos(latitude)を掛ける。
        meters_per_pixel = (
            2.0 * math.pi * 6378137.0 * math.cos(math.radians(latitude))
            / float((1 << zoom) * 256))
        target_pixels = min(170.0, max(64.0, (self.width() - 24.0) * 0.42))
        distance_m = self._nice_scale_distance(target_pixels * meters_per_pixel)
        bar_pixels = distance_m / meters_per_pixel
        label = ('{:g} km'.format(distance_m / 1000.0) if distance_m >= 1000.0
                 else '{:g} m'.format(distance_m))

        # タイトルと右上の操作ボタンを避け、地図画像の左上寄りに配置する。
        x, y = 10.0, 66.0
        box_width = bar_pixels + 20.0
        painter.fillRect(QRectF(x, y, box_width, 38.0), QColor(16, 24, 32, 220))
        painter.setPen(QColor(PALETTE['text']))
        painter.drawText(QRectF(x + 7.0, y + 1.0, box_width - 14.0, 16.0),
                         Qt.AlignLeft | Qt.AlignVCenter, label)
        bar_left = x + 8.0
        bar_right = bar_left + bar_pixels
        bar_y = y + 27.0
        painter.setPen(QPen(QColor(PALETTE['text']), 3))
        painter.drawLine(QPointF(bar_left, bar_y), QPointF(bar_right, bar_y))
        painter.drawLine(QPointF(bar_left, bar_y - 5.0), QPointF(bar_left, bar_y + 5.0))
        painter.drawLine(QPointF(bar_right, bar_y - 5.0), QPointF(bar_right, bar_y + 5.0))

    def _paint_geographic_view(self, painter):
        zoom = self._zoom
        center_x, center_y = self._world_pixel(
            self.geo_center[0], self.geo_center[1], zoom)
        left = center_x - self.width() / 2.0
        top = center_y - self.height() / 2.0
        tile_zoom = min(zoom, self.tile_max_zoom)
        enlargement = 1 << (zoom - tile_zoom)
        tile_pixels = 256.0 * enlargement
        first_x = int(math.floor(left / tile_pixels))
        last_x = int(math.floor((left + self.width() - 1) / tile_pixels))
        first_y = int(math.floor(top / tile_pixels))
        last_y = int(math.floor((top + self.height() - 1) / tile_pixels))
        tile_count = 1 << tile_zoom
        visible_tiles = [(tile_zoom, tile_x % tile_count, tile_y)
                         for tile_x in range(first_x, last_x + 1)
                         for tile_y in range(first_y, last_y + 1)
                         if 0 <= tile_y < tile_count]
        self._tile_provider.set_visible_tiles(visible_tiles)
        painter.fillRect(self.rect(), QColor('#d7d9d3'))
        painter.setPen(QPen(QColor('#b8beb7'), 1, Qt.DotLine))
        for x in range(0, self.width(), 64):
            painter.drawLine(x, 0, x, self.height())
        for y in range(0, self.height(), 64):
            painter.drawLine(0, y, self.width(), y)

        image_count = 0
        for tile_x in range(first_x, last_x + 1):
            for tile_y in range(first_y, last_y + 1):
                image = self._tile_provider.tile(tile_zoom, tile_x, tile_y)
                if image is not None:
                    image_count += 1
                    # scaled画像の巨大なallocationを避け、canvas clip内だけ描画する。
                    painter.drawImage(QRectF(tile_x * tile_pixels - left,
                                             tile_y * tile_pixels - top,
                                             tile_pixels, tile_pixels), image)

        projected = []
        for latitude, longitude in self.gnss_points:
            px, py = self._world_pixel(latitude, longitude, zoom)
            projected.append(QPointF(px - left, py - top))
        if len(projected) > 1:
            painter.setPen(QPen(QColor(PALETTE['blue']), 3))
            painter.drawPolyline(QPolygonF(projected))

        position_x, position_y = self._world_pixel(
            self.geo_position[0], self.geo_position[1], zoom)
        marker_x, marker_y = position_x - left, position_y - top
        marker_color = {'NO FIX': PALETTE['red'], 'GNSS': PALETTE['yellow'],
                        'RTK FLOAT': '#ff9f1c', 'RTK FIX': PALETTE['green']}.get(
                            self.fix_state, PALETTE['muted'])
        painter.save()
        painter.translate(marker_x, marker_y)
        if self.geo_yaw is None:
            painter.setPen(QPen(QColor(PALETTE['text']), 2))
            painter.setBrush(QColor(marker_color))
            painter.drawEllipse(QPointF(0, 0), 8, 8)
        else:
            painter.rotate(-math.degrees(self.geo_yaw))
            painter.setPen(QPen(QColor(PALETTE['text']), 2))
            painter.setBrush(QColor(marker_color))
            painter.drawPolygon(QPolygonF([
                QPointF(-11, 9), QPointF(12, 0), QPointF(-11, -9),
            ]))
        painter.restore()

        painter.setPen(QColor(PALETTE['text']))
        source_text = self._tile_provider.source_summary
        painter.fillRect(QRectF(6, 6, min(self.width() - 12, 270), 24),
                         QColor(16, 24, 32, 210))
        if enlargement > 1:
            source_text += ' / 画像×{}'.format(enlargement)
        painter.drawText(12, 23, '{}  z{}'.format(source_text, zoom))
        attribution = self._tile_provider.attribution
        attribution_width = min(self.width() - 12, len(attribution) * 8 + 12)
        painter.fillRect(QRectF(self.width() - attribution_width - 6,
                                self.height() - 25, attribution_width, 20),
                         QColor(16, 24, 32, 210))
        painter.drawText(self.width() - attribution_width,
                         self.height() - 10, attribution)
        self._paint_scale_bar(painter, zoom)
        if image_count == 0:
            painter.drawText(self.rect(), Qt.AlignCenter,
                             '地図tileなし\nGNSS座標と軌跡を表示中')

    def paintEvent(self, _event):
        painter = QPainter(self)
        if self.geo_center is not None:
            self._paint_geographic_view(painter)
            return
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
        # PyQt5のQPolygonFには座標tupleでなくQPointFを渡す。
        painter.drawPolygon(QPolygonF([
            QPointF(-11, 9), QPointF(12, 0), QPointF(-11, -9),
        ]))
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
        self._latest_frame = None
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(4)
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
        self.image.setMinimumSize(260, 130)
        self.image.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.image.setStyleSheet('background:#080d12; color:' + PALETTE['muted'] + ';')
        layout.addWidget(self.image, 1)
        self.setMinimumSize(300, 250)

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
            self._latest_frame = None
            if not camera['enabled']:
                self.image.setPixmap(QPixmap())
                self.image.setText('カメラ表示はOFFです')
            elif camera['error']:
                self.image.setText('画像の変換に失敗しました')
            else:
                self.image.setText('画像待受中')
            return
        self._latest_frame = frame
        if camera['converted_count'] == self._last_rendered_count:
            return
        self._render_frame(frame)
        self._last_rendered_count = camera['converted_count']

    def _render_frame(self, frame):
        """現在の表示枠に合わせてRGB frameを縮小し、Qt imageへコピーする。"""
        height, width, channels = frame.shape
        qimage = QImage(frame.data, width, height, channels * width, QImage.Format_RGB888).copy()
        pixmap = QPixmap.fromImage(qimage).scaled(
            self.image.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.image.setPixmap(pixmap)
        self.image.setText('')
        self._qt_render_count += 1

    def resizeEvent(self, event):
        # 画面サイズ変更時にも最後に受信したframeを新しい表示枠へ再描画する。
        super().resizeEvent(event)
        camera_image = getattr(self, '_latest_frame', None)
        if camera_image is not None:
            self._render_frame(camera_image)


def _circle_diameter(widget):
    """左右の表示枠の小さい方に合わせ、許可ボタンとスティック外円を同径にする。"""
    reference = getattr(widget, 'circle_reference', None)
    sizes = [widget.width(), widget.height()]
    if reference is not None:
        sizes.extend([reference.width(), reference.height()])
    return max(0.0, min(sizes)-14.0)


class VehicleEnableButton(QPushButton):
    """走行許可の円形ボタン。回転中ではなく制御launch稼働中をSTOPで示す。"""

    def __init__(self):
        super().__init__('WAIT')
        self.color = PALETTE['muted']
        self.circle_reference = None
        self.setMinimumSize(88, 88)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setEnabled(False)

    def hitButton(self, pos):
        delta = QPointF(pos)-QPointF(self.width()/2.0, self.height()/2.0)
        return delta.x()**2+delta.y()**2 <= (_circle_diameter(self)/2.0)**2

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        radius = _circle_diameter(self)/2.0
        center = QPointF(self.width()/2.0, self.height()/2.0)
        color = QColor(self.color)
        if not self.isEnabled():
            color.setAlpha(135)
        painter.setPen(QPen(QColor(PALETTE['text']) if self.hasFocus()
                            else QColor(PALETTE['muted']), 2))
        painter.setBrush(color.darker(120) if self.isDown() else color)
        painter.drawEllipse(center, radius, radius)
        font = QFont(self.font())
        font.setPointSize(20)
        font.setBold(True)
        painter.setFont(font)
        painter.setPen(QColor(PALETTE['background']))
        painter.drawText(self.rect(), Qt.AlignCenter, self.text())


class BatteryGauge(QWidget):
    """電池外形と10区画の残量を描く。未受信は灰色で、空電池と区別する。"""

    def __init__(self):
        super().__init__()
        self.percent = None
        self.color = PALETTE['muted']
        self.setMinimumSize(50, 60)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def paintEvent(self, event):
        painter = QPainter(self)
        width = max(24.0, min(58.0, self.width()-16.0))
        height = max(30.0, min(110.0, self.height()-14.0))
        body = QRectF((self.width()-width)/2.0, (self.height()-height)/2.0+4.0,
                      width, height)
        color = QColor(self.color)
        painter.setPen(QPen(color, 2))
        painter.setBrush(QColor(PALETTE['background']))
        painter.drawRect(body)
        painter.fillRect(QRectF(body.center().x()-width*0.15, body.top()-6.0,
                                width*0.3, 6.0), color)
        inner = body.adjusted(5.0, 5.0, -5.0, -5.0)
        slot = inner.height()/10.0
        for index in range(10):
            rect = QRectF(inner.left(), inner.bottom()-(index+1)*slot+1.0,
                          inner.width(), max(1.0, slot-2.0))
            painter.fillRect(rect, QColor('#344b5b'))
            if self.percent is not None:
                fraction = max(0.0, min(1.0, self.percent/10.0-index))
                filled = QRectF(rect.left(), rect.bottom()-rect.height()*fraction,
                                rect.width(), rect.height()*fraction)
                painter.fillRect(filled, color)
        if self.percent is None:
            painter.setPen(QColor(PALETTE['text']))
            painter.drawText(body, Qt.AlignCenter, '?')


class BatteryPanel(QGroupBox):
    """左下の共通電源表示。電圧警告はSOCと独立し、staleを残量ゼロにしない。"""

    def __init__(self, config):
        super().__init__('BATTERY')
        battery = config.get('battery', {})
        self.stale_timeout = float(battery.get('stale_timeout_sec', 3.0))
        self.setMinimumWidth(96)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 4, 5, 4)
        layout.setSpacing(2)
        self.gauge = BatteryGauge()
        layout.addWidget(self.gauge, 1)
        self.percent = QLabel('≈ --%')
        self.percent.setAlignment(Qt.AlignCenter)
        font = QFont(self.percent.font())
        font.setPointSize(18)
        font.setBold(True)
        self.percent.setFont(font)
        layout.addWidget(self.percent)
        self.voltage = QLabel('--.- V')
        self.voltage.setAlignment(Qt.AlignCenter)
        self.voltage.setStyleSheet('font-size:14px; font-weight:bold;')
        layout.addWidget(self.voltage)
        self.status = QLabel('データ待受')
        self.status.setAlignment(Qt.AlignCenter)
        self.status.setWordWrap(True)
        self.status.setStyleSheet('font-size:11px;')
        layout.addWidget(self.status)
        model = QLabel('{}\n18V ×{}並列'.format(
            battery.get('model', 'BL1860B'), battery.get('parallel_packs', 2)))
        model.setAlignment(Qt.AlignCenter)
        model.setStyleSheet('font-size:9px; color:'+PALETTE['muted'])
        layout.addWidget(model)
        self.setToolTip('停止中の電圧と走行中の正のIbusによる参考残量です。負電流は無視します。\n'
                        '共通bus電圧だけでは各バッテリ・各セルの残量や安全性は判断できません。\n'
                        '走行許可OFFなどで/motor_stateが止まるとSTALEになります。')

    def refresh(self, snapshot):
        item = snapshot.get('telemetry', {}).get('battery')
        age = item.get('age') if item else None
        fresh = age is not None and age <= self.stale_timeout
        value = item['value'] if fresh else {}
        percent = value.get('percent')
        voltage = value.get('voltage_v')
        level = value.get('level', 'STALE' if item else 'WAITING')
        colors = {'REFERENCE': PALETTE['green'], 'LOW': PALETTE['yellow'],
                  'CRITICAL': PALETTE['red'], 'HIGH': PALETTE['red']}
        color = colors.get(level, PALETTE['muted'])
        if level == 'REFERENCE' and percent is None:
            color = PALETTE['muted']
        self.percent.setText('≈ {:.0f}%'.format(percent) if percent is not None else '≈ --%')
        self.percent.setStyleSheet('font-size:18pt; font-weight:bold; color:'+color)
        self.voltage.setText('{:.2f} V'.format(voltage) if voltage is not None else '--.- V')
        self.status.setText({
            'REFERENCE': {
                'COULOMB_COUNTING': '走行中 / 電流積算',
                'WAIT_BASELINE': '停止基準待ち', 'CURRENT_INVALID': '電流無効 / 停止待ち',
                'WAIT_SETTLE': '停止整定待ち',
                'REST_ESTIMATE': '停止電圧の概算',
            }.get(value.get('estimate_mode'), '電圧から概算'), 'LOW': '低電圧 / 交換目安',
            'CRITICAL': '使用中断目安', 'HIGH': '電圧範囲外',
            'INVALID': '電圧値が無効', 'STALE': 'STALE', 'WAITING': 'データ待受',
        }.get(level, 'データ待受'))
        self.status.setStyleSheet('font-size:11px; color:'+color)
        tooltip = '最終受信から {:.1f}秒'.format(age) if age is not None else '未受信'
        anchor = value.get('anchor_voltage_v')
        if anchor is not None:
            tooltip += '\n基準電圧 {:.2f} V / 容量 {:.1f} Ah'.format(
                anchor, value.get('capacity_ah', 0.0))
            tooltip += '\n基準更新後の走行消費 {:.4f} Ah'.format(value.get('consumed_ah', 0.0))
        current = value.get('current_a')
        if current is not None:
            tooltip += '\nIbus {:.2f} A（負値は積算に使用しない）'.format(current)
        self.status.setToolTip(tooltip)
        self.gauge.percent, self.gauge.color = percent, color
        self.gauge.update()


class TelemetryPanel(QWidget):
    def __init__(self, config=None, backend=None):
        super().__init__()
        config = config or {}
        self.backend = backend
        self._vehicle_active = False
        root = QHBoxLayout(self)
        self.battery_panel = BatteryPanel(config)
        attitude = QGroupBox('ATTITUDE')
        left = QVBoxLayout(attitude)
        self.canvas = AttitudeCanvas(config.get('attitude', {}))
        left.addWidget(self.canvas)
        self.attitude = QLabel('roll --   pitch --\nyaw --')
        self.attitude.setWordWrap(True)
        left.addWidget(self.attitude)
        joystick_config = config.get('joystick', {})
        joystick_topic = config.get('topics', {}).get('joy', '/pm/joy')
        joystick = QGroupBox('JOYSTICK / TELEOP  ' + joystick_topic)
        joystick_columns = QHBoxLayout(joystick)
        joystick_columns.setContentsMargins(6, 4, 6, 4)
        joystick_columns.setSpacing(6)
        enable_panel = QWidget()
        enable_layout = QVBoxLayout(enable_panel)
        enable_layout.setContentsMargins(0, 0, 0, 0)
        enable_layout.setSpacing(2)
        title = QLabel('MOTOR ENABLE')
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet('font-size:16.5px; font-weight:bold;')
        enable_layout.addWidget(title)
        self.vehicle_button = VehicleEnableButton()
        self.vehicle_button.clicked.connect(self._vehicle_clicked)
        enable_layout.addWidget(self.vehicle_button, 1)
        self.vehicle_status = QLabel('Manager待受中')
        self.vehicle_status.setAlignment(Qt.AlignCenter)
        self.vehicle_status.setWordWrap(True)
        self.vehicle_status.setStyleSheet('font-size:11px;')
        enable_layout.addWidget(self.vehicle_status)
        indicator_panel = QWidget()
        joystick_columns.addWidget(enable_panel, 1)
        joystick_columns.addWidget(indicator_panel, 1)
        right = QVBoxLayout(indicator_panel)
        right.setContentsMargins(6, 4, 6, 4)
        right.setSpacing(2)
        self.mode = QLabel('NO DATA')
        self.mode.setAlignment(Qt.AlignCenter)
        self.mode.setStyleSheet('font-size:16.5px; font-weight:bold; color:' + PALETTE['muted'])
        right.addWidget(self.mode)
        self.stick = JoystickCanvas()
        self.stick.circle_reference = self.vehicle_button
        self.vehicle_button.circle_reference = self.stick
        self.stick.set_deadzone(float(joystick_config.get('deadzone', 0.0)))
        right.addWidget(self.stick, 1)
        self.speed = QLabel('--.- km/h')
        self.speed.setAlignment(Qt.AlignCenter)
        speed_font = QFont(self.speed.font())
        speed_font.setPointSize(20)
        self.speed.setFont(speed_font)
        right.addWidget(self.speed)
        self.stick_mapping = QLabel('joystick設定を読み込み中')
        self.stick_mapping.setAlignment(Qt.AlignCenter)
        self.stick_mapping.setWordWrap(True)
        self.stick_mapping.setStyleSheet('font-size:9px; color:' + PALETTE['muted'])
        right.addWidget(self.stick_mapping)
        axis_linear = int(joystick_config.get('axis_linear_x', -1))
        axis_angular = int(joystick_config.get('axis_angular_yaw', -1))
        enable_button = int(joystick_config.get('enable_button', -1))
        turbo_button = int(joystick_config.get('enable_turbo_button', -1))
        display_signs = joystick_config.get('axis_display_sign', {})
        display_sign_x = -1.0 if float(display_signs.get('x', -1.0)) < 0.0 else 1.0
        display_sign_y = -1.0 if float(display_signs.get('y', 1.0)) < 0.0 else 1.0
        self.joystick_settings = {
            'axis_linear_x': axis_linear,
            'axis_angular_yaw': axis_angular,
            'display_sign_x': display_sign_x,
            'display_sign_y': display_sign_y,
            'enable_button': enable_button,
            'enable_turbo_button': turbo_button,
            'require_enable_button': bool(
                joystick_config.get('require_enable_button', False)),
            'config_error': joystick_config.get('config_error', ''),
        }
        config_error = joystick_config.get('config_error', '')
        self.stick_mapping.setToolTip(joystick_config.get('config_source', config_error))
        if config_error:
            self.stick_mapping.setText('設定読込エラー: ' + config_error)
            self.stick_mapping.setStyleSheet('font-size:9px; color:' + PALETTE['red'])
        else:
            self.stick_mapping.setText(
                'X axis {} ({:+.0f}) / Y axis {} ({:+.0f})  ·  enable B{} / turbo B{}'.format(
                    axis_angular, display_sign_x, axis_linear, display_sign_y,
                    enable_button, turbo_button))
        self.splitter = QSplitter(Qt.Horizontal)
        self.splitter.setChildrenCollapsible(False)
        self.splitter.setHandleWidth(6)
        # 左端に電池表示を追加し、姿勢/操縦の幅を少しずつ譲る。全体のresizeは維持する。
        self.splitter.addWidget(self.battery_panel)
        self.splitter.addWidget(attitude)
        self.splitter.addWidget(joystick)
        self.splitter.setStretchFactor(0, 1)
        self.splitter.setStretchFactor(1, 3)
        self.splitter.setStretchFactor(2, 4)
        root.addWidget(self.splitter)

    def _vehicle_clicked(self):
        if self.backend is None:
            return
        if self._vehicle_active:
            self.backend.request_vehicle_stop()
        elif QMessageBox.question(
                self, '走行許可の確認',
                'モーター制御とjoystick速度指令を有効にします。\n'
                '車両周囲の安全を確認し、スティックを中立にしてください。',
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No) == QMessageBox.Yes:
            self.backend.request_vehicle_start()

    def _refresh_vehicle(self, snapshot):
        vehicle = snapshot.get('vehicle', {})
        state = vehicle.get('state', 'UNKNOWN')
        connected = snapshot.get('connection') == 'CONNECTED'
        available = connected and snapshot.get('vehicle_services_ready', False)
        active = state == 'RUNNING' or vehicle.get('unit_state') == 'active'
        self._vehicle_active = active
        transitioning = state in ('STARTING', 'STOPPING')
        label = ('WAIT' if state == 'UNKNOWN' else
                 'WAIT' if transitioning else 'STOP' if active else 'START')
        self.vehicle_button.setText(label)
        self.vehicle_button.color = (PALETTE['yellow'] if transitioning else
                                     PALETTE['red'] if active else
                                     PALETTE['green'] if state == 'STOPPED' else PALETTE['muted'])
        self.vehicle_button.setEnabled(
            self.backend is not None and available and not transitioning
            and state != 'UNKNOWN'
            and (active or snapshot.get('core_state') == 'RUNNING'))
        detail = vehicle.get('detail', {})
        if not connected:
            text = 'Manager通信待受中'
        elif vehicle.get('error'):
            text = vehicle['error']
        elif active:
            text = ('車両状態 STALE' if not vehicle.get('fresh') else
                    '緊急停止中' if detail.get('emergency_stop') else
                    'ODrive接続待ち' if not detail.get('connected') else
                    '走行許可 ON' if detail.get('armed') else '中立指令待ち')
        else:
            text = '走行許可 OFF' if state == 'STOPPED' else state
        # 長いエラーはtooltipへ。小タイルの最小幅を増やさない。
        self.vehicle_status.setText(text if len(text) <= 35 else text[:32]+'…')
        self.vehicle_status.setToolTip(text)
        self.vehicle_button.setToolTip('STOPは走行用nodeの停止です。非常停止とは別の操作です。')
        self.vehicle_button.update()

    @staticmethod
    def _fresh(snapshot, key):
        item = snapshot['telemetry'].get(key)
        return item['value'] if item and item['fresh'] else None

    def refresh(self, snapshot, config):
        self.battery_panel.refresh(snapshot)
        self._refresh_vehicle(snapshot)
        odom = self._fresh(snapshot, 'odometry')
        if odom:
            roll, pitch, yaw = odom['roll'], odom['pitch'], odom['yaw']
            self.attitude.setText(
                'Roll: {:+.1f}°  Pitch: {:+.1f}°\n'
                'Odom yaw: {:+.1f}°（地球方位ではない）'.format(
                    math.degrees(roll), math.degrees(pitch), math.degrees(yaw),
                ))
            self.canvas.set_attitude(roll, pitch, yaw)
        else:
            self.attitude.setText('姿勢データ stale / 待受中')
        # Odom姿勢と磁気コンパスを明示的に分離。未受信を「東0度」にしない。
        from .magnetic_heading import cardinal
        heading = self._fresh(snapshot, 'magnetic_heading')
        if heading is None:
            compass_text = '磁気方位: stale / 待受中'
        elif heading['bearing'] is None:
            compass_text = '磁気方位: 無効 / ' + heading['error']
        else:
            compass_text = '磁気方位: {:.1f}° {}（{}）'.format(
                heading['bearing'], cardinal(heading['bearing']),
                '補正設定済' if heading['calibrated'] else '未校正・参考値')
        self.attitude.setText(self.attitude.text()+'\n'+compass_text)
        joy = self._fresh(snapshot, 'joy')
        joy_cfg = self.joystick_settings
        if joy_cfg['config_error']:
            mode = 'CONFIG ERROR'
            x_value = y_value = None
        elif joy is None:
            mode = 'NO DATA'
            x_value = y_value = None
        else:
            buttons = joy['buttons']
            enable_i = joy_cfg['enable_button']
            turbo_i = joy_cfg['enable_turbo_button']
            enable_pressed = (0 <= enable_i < len(buttons) and buttons[enable_i] != 0)
            turbo = (0 <= turbo_i < len(buttons) and buttons[turbo_i] != 0)
            enabled = not joy_cfg['require_enable_button'] or enable_pressed
            # teleop_twist_joyと同じ優先順。Turboボタン単独でも速度指令が有効になり、
            # 通常enableボタンとの同時押しを必要としない。
            mode = 'TURBO' if turbo else 'ENABLED' if enabled else 'DISABLED'
            raw_x = self._joy_axis(joy['axes'], joy_cfg['axis_angular_yaw'])
            raw_y = self._joy_axis(joy['axes'], joy_cfg['axis_linear_x'])
            # 表示符号だけ補正し、ROS Joy messageとteleop指令には手を加えない。
            x_value = None if raw_x is None else raw_x * joy_cfg['display_sign_x']
            y_value = None if raw_y is None else raw_y * joy_cfg['display_sign_y']
        colors = {'DISABLED': PALETTE['green'], 'ENABLED': '#ff9f1c',
                  'TURBO': PALETTE['red'], 'NO DATA': PALETTE['muted'],
                  'CONFIG ERROR': PALETTE['red']}
        self.mode.setText(mode)
        mode_color = colors.get(mode, PALETTE['muted'])
        self.mode.setStyleSheet('font-size:16.5px; font-weight:bold; color:' + mode_color)
        self.stick.set_input(x_value, y_value, mode_color, mode)
        # vxの符号を保持してm/sからkm/hへ換算する。後退は負値で示す。
        # stale/非有限値を0 km/hと誤表示せず、受信待ち表示へ戻す。
        wheel_odometry = self._fresh(snapshot, 'wheel_odometry')
        vx = wheel_odometry.get('vx') if wheel_odometry else None
        self.speed.setText('{:.1f} km/h'.format(vx * 3.6)
                           if vx is not None and math.isfinite(vx) else '--.- km/h')
        self.speed.setStyleSheet('font-weight:bold; color:' + mode_color)
        self.speed.setToolTip('ホイールodometry vx × 3.6（後退は負値）'
                              if vx is not None else 'ホイールodometry受信待ち / STALE')

    @staticmethod
    def _joy_axis(axes, index):
        """Joy.axesのindexを検査し、表示用に[-1, 1]へ制限する。"""
        if index < 0 or index >= len(axes):
            return None
        value = float(axes[index])
        if not math.isfinite(value):
            return None
        return max(-1.0, min(1.0, value))


class JoystickCanvas(QWidget):
    """joy_teleopが参照する2軸を円形領域上のstick位置として描く。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.x_value = None
        self.y_value = None
        self.color = PALETTE['muted']
        self.mode = 'NO DATA'
        self.deadzone = 0.0
        self.setMinimumSize(88, 88)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_deadzone(self, value):
        self.deadzone = max(0.0, min(1.0, float(value)))

    def set_input(self, x_value, y_value, color, mode='NO DATA'):
        self.x_value = x_value
        self.y_value = y_value
        self.color = color
        self.mode = mode
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        diameter = _circle_diameter(self)
        if diameter <= 0:
            return
        radius = diameter * 0.5
        center_x, center_y = self.width() * 0.5, self.height() * 0.5
        painter.setPen(QPen(QColor(PALETTE['muted']), 2))
        painter.setBrush(QColor('#101820'))
        painter.drawEllipse(QPointF(center_x, center_y), radius, radius)
        painter.setPen(QPen(QColor('#344b5b'), 1))
        painter.drawLine(QPointF(center_x - radius, center_y),
                         QPointF(center_x + radius, center_y))
        painter.drawLine(QPointF(center_x, center_y - radius),
                         QPointF(center_x, center_y + radius))
        if self.deadzone > 0.0:
            painter.setPen(QPen(QColor(PALETTE['muted']), 1, Qt.DotLine))
            deadzone_radius = radius * self.deadzone
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(QPointF(center_x, center_y),
                               deadzone_radius, deadzone_radius)

        x_value = self.x_value if self.x_value is not None else 0.0
        y_value = self.y_value if self.y_value is not None else 0.0
        travel = radius * 0.82
        # 表示座標はX正を右、Y正を上とし、Qt画面の下向きYに合わせて上下を反転する。
        stick_x = center_x + x_value * travel
        stick_y = center_y - y_value * travel
        painter.setPen(QPen(QColor(PALETTE['text']), 2))
        painter.setBrush(QColor(self.color))
        painter.drawEllipse(QPointF(stick_x, stick_y), 21.0, 21.0)
        # 色に加えてmodeを形でも示す。円の中心に右向きの再生/早送り記号を描く。
        # font glyphを使わずpolygonにして、日本語font等の選択に依存させない。
        if self.mode in ('ENABLED', 'TURBO'):
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(PALETTE['background']))
            offsets = (-11.0, 1.0) if self.mode == 'TURBO' else (-5.0,)
            for offset in offsets:
                left = stick_x + offset
                painter.drawPolygon(QPolygonF([
                    QPointF(left, stick_y - 8.0),
                    QPointF(left + 10.0, stick_y),
                    QPointF(left, stick_y + 8.0),
                ]))


class LocalizationPanel(QWidget):
    def __init__(self, config=None):
        super().__init__()
        map_config = (config or {}).get('map', {})
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(4)
        title = QLabel('MAP / GNSS POSITION')
        title.setStyleSheet('color:' + PALETTE['blue'] + '; font-weight:bold;')
        layout.addWidget(title)
        self.map = TrajectoryCanvas(map_config)
        layout.addWidget(self.map, 1)

        # 操作部品はcanvasの子widgetにし、地図画像の右上へ重ねて配置する。
        self.map_controls = QWidget(self.map)
        controls_layout = QHBoxLayout(self.map_controls)
        controls_layout.setContentsMargins(6, 6, 6, 6)
        controls_layout.setSpacing(4)
        self.follow_button = QPushButton('追従')
        self.follow_button.setCheckable(True)
        self.follow_button.setToolTip('現在の車両位置を地図中央に戻す')
        self.follow_button.setFixedSize(96, 44)
        self.zoom_out_button = QPushButton('ー')
        self.zoom_out_button.setToolTip('地図を縮小')
        self.zoom_out_button.setFixedSize(42, 44)
        self.zoom_level = QLabel()
        self.zoom_level.setAlignment(Qt.AlignCenter)
        self.zoom_level.setFixedSize(42, 44)
        self.zoom_level.setStyleSheet(
            'background:' + PALETTE['background'] + '; color:' + PALETTE['text']
            + '; font-size:14px; font-weight:bold;')
        self.zoom_in_button = QPushButton('＋')
        self.zoom_in_button.setToolTip('地図を拡大')
        self.zoom_in_button.setFixedSize(42, 44)
        for button in (self.zoom_out_button, self.zoom_in_button):
            button.setStyleSheet(
                'background:' + PALETTE['panel'] + '; color:' + PALETTE['text']
                + '; border:1px solid ' + PALETTE['muted']
                + '; font-size:24px; font-weight:bold;')
        self.follow_button.setStyleSheet(
            'background:' + PALETTE['panel'] + '; color:' + PALETTE['green']
            + '; border:1px solid ' + PALETTE['muted']
            + '; font-size:16px; font-weight:bold;')
        controls_layout.addWidget(self.follow_button)
        controls_layout.addWidget(self.zoom_out_button)
        controls_layout.addWidget(self.zoom_level)
        controls_layout.addWidget(self.zoom_in_button)
        self.map.set_controls_overlay(self.map_controls)
        self.follow_button.clicked.connect(self.map.follow_vehicle)
        self.map.follow_changed.connect(self._update_follow_button)
        self.zoom_out_button.clicked.connect(lambda: self.map.set_zoom(self.map.zoom - 1))
        self.zoom_in_button.clicked.connect(lambda: self.map.set_zoom(self.map.zoom + 1))
        self.map.zoom_changed.connect(self._update_zoom_controls)
        self._update_follow_button(True)
        self._update_zoom_controls(self.map.zoom)
        self.position = QLabel('位置: 待受中')
        self.gnss = QLabel('GNSS: NO FIX')
        self.position.setWordWrap(True)
        layout.addWidget(self.position)
        layout.addWidget(self.gnss)
        self.setMinimumSize(300, 220)

    def _update_zoom_controls(self, zoom):
        self.zoom_level.setText('z{}'.format(zoom))
        self.zoom_out_button.setEnabled(zoom > 1)
        self.zoom_in_button.setEnabled(zoom < self.map.max_zoom)

    def _update_follow_button(self, following):
        self.follow_button.setChecked(following)
        self.follow_button.setEnabled(self.map.geo_position is not None)
        color = PALETTE['green'] if following else PALETTE['yellow']
        self.follow_button.setStyleSheet(
            'background:' + PALETTE['panel'] + '; color:' + color
            + '; border:1px solid ' + PALETTE['muted']
            + '; font-size:16px; font-weight:bold;')

    def refresh(self, snapshot):
        telemetry = snapshot['telemetry']
        odom_item = telemetry.get('odometry')
        navpvt_item = telemetry.get('navpvt')
        fix_item = telemetry.get('fix')
        navpvt = navpvt_item['value'] if navpvt_item and navpvt_item['fresh'] else None
        fix = fix_item['value'] if fix_item and fix_item['fresh'] else None
        if navpvt is not None:
            state = navpvt['gnss_state']
        elif fix is not None:
            state = 'GNSS' if fix['status'] >= 0 else 'NO FIX'
        else:
            state = 'STALE / NO DATA'
        self.gnss.setText('GNSS: ' + state)
        color = {'NO FIX': PALETTE['red'], 'GNSS': PALETTE['yellow'],
                 'RTK FLOAT': '#ff9f1c', 'RTK FIX': PALETTE['green'],
                 'STALE / NO DATA': PALETTE['muted']}.get(state, PALETTE['muted'])
        self.gnss.setStyleSheet('font-weight:bold; color:' + color)
        odom = odom_item['value'] if odom_item and odom_item['fresh'] else None
        if odom is not None:
            self.map.add_pose(odom['x'], odom['y'], odom['yaw'], state)
            self.position.setText('map ({})  x: {:.2f} m  y: {:.2f} m  z: {:.2f} m'.format(
                odom['frame_id'] or 'unknown frame', odom['x'], odom['y'], odom['z']))
        else:
            self.position.setText('odometry stale / 待受中')
        valid_fix = (fix is not None and fix['status'] >= 0
                     and math.isfinite(fix['latitude']) and math.isfinite(fix['longitude'])
                     and abs(fix['latitude']) <= 90.0 and abs(fix['longitude']) <= 180.0)
        if valid_fix:
            self.position.setText(self.position.text() +
                '    WGS84: {:.7f}, {:.7f}'.format(fix['latitude'], fix['longitude']))
            self.map.set_geographic_view(
                (fix['latitude'], fix['longitude']), snapshot.get('gnss_path', []),
                odom['yaw'] if odom is not None else None, state)
        else:
            # GNSSがないときはWeb地図へ誤った座標を置かず、local map axesへ戻す。
            self.map.set_geographic_view(None, [], None, state)


class RecordingPanel(QWidget):
    """Jetson上のmission/debug recorder unitを操作・監視する。"""

    def __init__(self, backend, config):
        super().__init__()
        self.backend = backend
        self.config = config.get('recording', {})
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)
        layout.setSpacing(2)
        title = QLabel('JETSON ROS BAG RECORDING')
        title.setStyleSheet('color:' + PALETTE['blue'] + '; font-size:11px; font-weight:bold;')
        layout.addWidget(title)
        self.directory = QLabel('')
        self.disk = QLabel('')
        # 長いbag名がタイルの最小幅を押し広げないよう、幅制約内で折り返す。
        self.directory.setWordWrap(True)
        self.directory.setTextFormat(Qt.PlainText)
        self.directory.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self.directory.setMinimumWidth(0)
        self.external = QLabel('')
        for label in (self.directory, self.disk, self.external):
            label.setStyleSheet('font-size:10px;')
        info = QGridLayout()
        info.setContentsMargins(0, 0, 0, 0)
        info.setHorizontalSpacing(6)
        info.setVerticalSpacing(1)
        info.addWidget(self.directory, 0, 0, 1, 2)
        info.addWidget(self.disk, 1, 0)
        info.addWidget(self.external, 1, 1)
        layout.addLayout(info)
        self.rows = {}
        profiles = self.config.get('profiles', {})
        for profile in ('mission', 'debug'):
            profile_widget = QWidget()
            profile_layout = QVBoxLayout(profile_widget)
            profile_layout.setContentsMargins(0, 0, 0, 0)
            profile_layout.setSpacing(0)
            row = QHBoxLayout()
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(3)
            name = QLabel(profile.upper())
            name.setStyleSheet('font-size:10px; font-weight:bold;')
            state = QLabel('STOPPED')
            detail = QLabel('')
            state.setStyleSheet('font-size:10px; font-weight:bold;')
            detail.setStyleSheet('font-size:9px; color:' + PALETTE['muted'] + ';')
            detail.setWordWrap(True)
            detail.setTextFormat(Qt.PlainText)
            detail.setMinimumWidth(0)
            detail.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
            toggle = QPushButton('▶  START')
            toggle.setMinimumSize(142, 38)
            toggle.clicked.connect(lambda _checked=False, p=profile: self._toggle_recording(p))
            row.addWidget(name)
            row.addWidget(state)
            row.addStretch(1)
            row.addWidget(toggle)
            profile_layout.addLayout(row)
            profile_layout.addWidget(detail)
            layout.addWidget(profile_widget)
            self.rows[profile] = (state, detail, toggle, profiles.get(profile, {}))
        self.warning = QLabel('')
        self.warning.setStyleSheet('font-size:10px; font-weight:bold; color:' + PALETTE['red'])
        layout.addWidget(self.warning)
        self.setMinimumSize(0, 0)

    def _start(self, profile):
        ok, message = self.backend.start_recording(profile)
        if not ok:
            self.warning.setText(message)

    def _toggle_recording(self, profile):
        """profile状態に応じて記録開始か正常停止のどちらかを実行する。"""
        state = self.rows[profile][0].text()
        if state == 'RECORDING':
            self.backend.stop_recording(profile)
        elif state == 'STOPPED':
            self._start(profile)

    def refresh(self, snapshot):
        # profile別の行にし、区切り位置に不可視の改行候補を入れる。
        # tooltipには元のパスを保持する。ROS/backendの値は変更しない。
        paths = [profile.upper()+': '+str(item.get('output'))
                 for profile in ('mission', 'debug')
                 for item in (snapshot['recordings'].get(profile) or {},)
                 if item.get('output')]
        directory_text = '\n'.join(paths) if paths else snapshot['recording_directory']
        self.directory.setText(directory_text.replace('/', '/\u200b').replace('_', '_\u200b'))
        self.directory.setToolTip(directory_text)
        free = snapshot['free_bytes']
        if free is None:
            self.disk.setText('Disk free: 不明')
            low = False
        else:
            free_gib = free / float(1024 ** 3)
            self.disk.setText('Disk free: {:.1f} GiB'.format(free_gib))
            low = free < int(float(self.config.get('low_disk_gib', 5.0)) * 1024 ** 3)
        warnings = []
        if low:
            warnings.append('Jetsonの空き容量が設定値を下回っています')
        if not snapshot['connection'] == 'CONNECTED':
            self.external.setText('Robot Manager接続待ち')
        else:
            self.external.setText('Mission/DebugをJetson上で管理')
        core_running = snapshot['core_state'] == 'RUNNING'
        for profile, (state, detail, toggle, _settings) in self.rows.items():
            item = snapshot['recordings'].get(profile)
            remote_state = item.get('state', 'UNKNOWN') if item else 'UNKNOWN'
            current = {
                'RUNNING': 'RECORDING', 'STOPPED': 'STOPPED',
                'STARTING': 'STARTING', 'STOPPING': 'STOPPING',
                'ERROR': 'ERROR',
            }.get(remote_state, 'UNKNOWN')
            state.setText(current)
            state.setStyleSheet('font-weight:bold; color:' +
                                (PALETTE['green'] if current == 'RECORDING' else
                                 PALETTE['red'] if current == 'ERROR' else
                                 PALETTE['yellow'] if current in ('STARTING', 'STOPPING') else
                                 PALETTE['muted']))
            if current == 'STOPPED':
                toggle_text, toggle_color = '▶  START', PALETTE['green']
                enabled = snapshot['connection'] == 'CONNECTED' and core_running
            elif current == 'RECORDING':
                toggle_text, toggle_color = '■  STOP', PALETTE['red']
                enabled = snapshot['connection'] == 'CONNECTED'
            elif current == 'STARTING':
                toggle_text, toggle_color, enabled = '…  STARTING', PALETTE['yellow'], False
            elif current == 'STOPPING':
                toggle_text, toggle_color, enabled = '…  STOPPING', PALETTE['yellow'], False
            elif current == 'ERROR':
                toggle_text, toggle_color, enabled = '!  CHECK BAG', PALETTE['red'], False
            else:
                toggle_text, toggle_color, enabled = 'STATUS UNKNOWN', PALETTE['muted'], False
            toggle.setText(toggle_text)
            toggle.setEnabled(enabled)
            toggle.setStyleSheet(
                'background:' + toggle_color + '; color:' + PALETTE['background']
                + '; border:1px solid ' + PALETTE['muted']
                + '; padding:5px 12px; font-size:14px; font-weight:bold;')
            if item:
                elapsed = max(0, int(float(item.get('elapsed_sec') or 0.0)))
                detail_text = '{}  /  {:02d}:{:02d}'.format(
                    item.get('output') or item.get('unit', ''),
                    elapsed // 60, elapsed % 60)
                item_error = item.get('error') or item.get('manager_error')
                if item_error:
                    warnings.append('{}: {}'.format(profile, item_error))
                detail.setText(detail_text.replace('/', '/\u200b').replace('_', '_\u200b'))
                detail.setToolTip('\n'.join((detail_text, item_error or '')))
            else:
                detail.setText('')
                detail.setToolTip('')
        self.warning.setText('\n'.join(warnings))
