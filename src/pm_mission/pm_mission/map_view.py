"""WGS84の経路編集canvas。画面pixelと緯度経度だけを相互変換する。"""
import math

from PyQt5.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QApplication, QWidget
from pm_ui_common.map_tiles import MapTileProvider

COLORS = {'start': '#7ac943', 'goal': '#ee6352', 'pass': '#4ea5d9',
          'pause': '#ff9f1c', 'confirm': '#bd93f9'}


def world_pixel(lat, lon, zoom):
    """Web Mercatorのworld pixel。緯度は投影可能な±85.051度に制限する。"""
    lat = max(-85.05112878, min(85.05112878, lat))
    size = 256.0 * 2**zoom
    sine = math.sin(math.radians(lat))
    return ((lon + 180) / 360 * size,
            (0.5 - math.log((1 + sine)/(1 - sine))/(4 * math.pi)) * size)


def geographic(x, y, zoom):
    """world pixelからWGS84へ戻す。日付変更線はwrapし、南北はclampする。"""
    size = 256.0 * 2**zoom
    y = max(0, min(size, y))
    return (math.degrees(math.atan(math.sinh(math.pi - 2 * math.pi * y / size))),
            (x % size) / size * 360 - 180)


class MissionMap(QWidget):
    """クリック追加・点drag移動・背景drag panを区別して経路変更を通知する。"""
    add_point = pyqtSignal(float, float)
    move_point = pyqtSignal(int, float, float)
    selection_changed = pyqtSignal(int)

    def __init__(self, config):
        super().__init__()
        self.provider = MapTileProvider(config, self)
        self.provider.tiles_changed.connect(self.update)
        self.center = (float(config.get('latitude', 0)), float(config.get('longitude', 0)))
        # tile取得上限と編集表示上限を分離する。上限を超えたtileは要求せず、
        # 既存画像だけを拡大し、点の座標変換は表示zoomの精度を維持する。
        self.tile_max_zoom = max(1, min(30, int(config.get('tile_max_zoom', 19))))
        self.max_zoom = max(self.tile_max_zoom, min(30, int(config.get('max_zoom', 26))))
        self.zoom = max(1, min(self.max_zoom, int(config.get('zoom', 3))))
        self.points = []
        self.selected = -1
        self.vehicle = None
        self.edit_mode = True
        self._press = self._last = None
        self._hit = -1
        self._dragged = False
        self.setMinimumSize(260, 240)
        self.setMouseTracking(True)

    def set_route(self, points, selected):
        self.points = points
        self.selected = selected
        self.update()

    def set_center(self, latitude, longitude):
        self.center = (max(-85.05112878, min(85.05112878, latitude)), longitude)
        self.update()

    def set_zoom(self, value):
        self.zoom = max(1, min(self.max_zoom, int(value)))
        self.update()

    def pixel(self, latitude, longitude):
        cx, cy = world_pixel(*self.center, self.zoom)
        x, y = world_pixel(latitude, longitude, self.zoom)
        size = 256.0 * 2**self.zoom
        dx = (x - cx + size/2) % size - size/2
        return QPointF(self.width()/2 + dx, self.height()/2 + y - cy)

    def coordinate(self, position):
        cx, cy = world_pixel(*self.center, self.zoom)
        return geographic(cx + position.x() - self.width()/2,
                          cy + position.y() - self.height()/2, self.zoom)

    def wheelEvent(self, event):
        if event.angleDelta().y():
            self.set_zoom(self.zoom + (1 if event.angleDelta().y() > 0 else -1))
            event.accept()

    def mousePressEvent(self, event):
        if event.button() not in (Qt.LeftButton, Qt.RightButton):
            return
        self._press = self._last = event.pos()
        self._dragged = False
        self._hit = -1
        if event.button() == Qt.LeftButton:
            for index in reversed(range(len(self.points))):
                point = self.points[index]
                pixel = self.pixel(point['latitude'], point['longitude'])
                if math.hypot(pixel.x()-event.x(), pixel.y()-event.y()) <= 14:
                    self._hit = index
                    self.selection_changed.emit(index)
                    break
        self.setCursor(Qt.ClosedHandCursor)

    def mouseMoveEvent(self, event):
        if self._press is None:
            return
        if (event.pos()-self._press).manhattanLength() >= QApplication.startDragDistance():
            self._dragged = True
        if self._dragged:
            if self.edit_mode and self._hit >= 0 and event.buttons() & Qt.LeftButton:
                self.move_point.emit(self._hit, *self.coordinate(event.pos()))
            else:
                delta = event.pos()-self._last
                cx, cy = world_pixel(*self.center, self.zoom)
                self.center = geographic(cx-delta.x(), cy-delta.y(), self.zoom)
                self.update()
        self._last = event.pos()

    def mouseReleaseEvent(self, event):
        if self._press is None:
            return
        if (not self._dragged and self._hit < 0 and self.edit_mode
                and event.button() == Qt.LeftButton):
            self.add_point.emit(*self.coordinate(event.pos()))
        self._press = self._last = None
        self.setCursor(Qt.ArrowCursor)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor('#d7d9d3'))
        cx, cy = world_pixel(*self.center, self.zoom)
        left, top = cx-self.width()/2, cy-self.height()/2
        tile_zoom = min(self.zoom, self.tile_max_zoom)
        enlargement = 2**(self.zoom-tile_zoom)
        tile_pixels = 256 * enlargement
        first_x, last_x = math.floor(left/tile_pixels), math.floor((left+self.width())/tile_pixels)
        first_y, last_y = math.floor(top/tile_pixels), math.floor((top+self.height())/tile_pixels)
        tiles = [(tile_zoom, x % 2**tile_zoom, y)
                 for x in range(first_x, last_x+1) for y in range(first_y, last_y+1)
                 if 0 <= y < 2**tile_zoom]
        self.provider.set_visible_tiles(tiles)
        painter.setPen(QPen(QColor('#aab0ac'), 1, Qt.DotLine))
        for x in range(0, self.width(), 64):
            painter.drawLine(x, 0, x, self.height())
        for y in range(0, self.height(), 64):
            painter.drawLine(0, y, self.width(), y)
        for x in range(first_x, last_x+1):
            for y in range(first_y, last_y+1):
                image = self.provider.tile(tile_zoom, x, y)
                if image is not None:
                    # 巨大なscaled QImageを生成せず、QPainterのclip付き描画で拡大する。
                    painter.drawImage(QRectF(x*tile_pixels-left, y*tile_pixels-top,
                                             tile_pixels, tile_pixels), image)
        painter.setRenderHint(QPainter.Antialiasing)
        pixels = [self.pixel(point['latitude'], point['longitude']) for point in self.points]
        if len(pixels) > 1:
            painter.setPen(QPen(QColor('#007fbb'), 3))
            painter.drawPolyline(QPolygonF(pixels))
        for index, pixel in enumerate(pixels):
            painter.setPen(QPen(QColor('#101820'), 3 if index == self.selected else 1))
            painter.setBrush(QColor(COLORS[self.points[index]['attribute']]))
            painter.drawEllipse(pixel, 12, 12)
            painter.drawText(int(pixel.x())+14, int(pixel.y())-10, str(index+1))
        if self.vehicle:
            pixel = self.pixel(*self.vehicle)
            painter.setPen(QPen(QColor('#101820'), 2))
            painter.setBrush(QColor('#f4d35e'))
            painter.drawRect(int(pixel.x())-5, int(pixel.y())-5, 10, 10)
        # 地上距離は中心緯度のcosを掛ける。1/2/5系列のscaleを描く。
        resolution = 2 * math.pi * 6378137 * math.cos(math.radians(self.center[0])) / (256*2**self.zoom)
        target = resolution * 100
        exponent = 10**math.floor(math.log10(target))
        factor = 5 if target/exponent >= 5 else 2 if target/exponent >= 2 else 1
        distance = exponent * factor
        width = distance / resolution
        painter.fillRect(8, 8, int(width)+30, 45, QColor(16, 24, 32, 210))
        painter.setPen(QColor('#f0f3bd'))
        painter.drawText(14, 26, '{:g} {}'.format(distance/1000 if distance >= 1000 else distance,
                                                'km' if distance >= 1000 else 'm'))
        painter.drawLine(QPointF(14, 41), QPointF(14+width, 41))
        painter.fillRect(0, self.height()-26, self.width(), 26, QColor(16, 24, 32, 220))
        zoom_text = 'Z{}'.format(self.zoom)
        if enlargement > 1:
            zoom_text += ' / tile Z{} ×{}（画像拡大）'.format(tile_zoom, enlargement)
        painter.drawText(8, self.height()-8, self.provider.attribution+' | '
                         +self.provider.source_summary+' | '+zoom_text)
