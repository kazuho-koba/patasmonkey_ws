"""現在表示中の範囲だけを取得・cacheするXYZ map tile provider。"""

import email.utils
import json
import os
import re
import time

from PyQt5.QtCore import QObject, QUrl, pyqtSignal
from PyQt5.QtGui import QImage
from PyQt5.QtNetwork import QNetworkAccessManager, QNetworkReply, QNetworkRequest


class MapTileProvider(QObject):
    """Qt event loopを止めずにonline XYZ tileとlocal cacheを返す。"""

    tiles_changed = pyqtSignal()
    FALLBACK_TTL_SEC = 7 * 24 * 60 * 60

    def __init__(self, config, parent=None):
        super().__init__(parent)
        self.url_template = str(config.get(
            'tile_url_template', 'https://tile.openstreetmap.org/{z}/{x}/{y}.png'))
        self.user_agent = str(config.get(
            'user_agent', 'PatasmonkeyOperatorConsole/0.1'))
        self.online_enabled = bool(config.get('online_tiles_enabled', True))
        self.cache_directory = os.path.abspath(os.path.expanduser(str(config.get(
            'tile_cache_dir', '~/.cache/pm_gui/tiles'))))
        offline_dir = config.get('offline_tiles_dir', '')
        self.offline_directory = (os.path.abspath(os.path.expanduser(str(offline_dir)))
                                  if offline_dir else '')
        self.attribution = str(config.get(
            'attribution', '© OpenStreetMap contributors'))
        self._network = QNetworkAccessManager(self)
        self._network.finished.connect(self._request_finished)
        self._pending = []
        self._active = {}
        self._images = {}
        self._sources = {}
        self._errors = set()
        self._retry_after = {}
        self._visible = set()
        self._cache_available = True
        try:
            os.makedirs(self.cache_directory, exist_ok=True)
        except OSError:
            # cache directoryが書けなくても、local tilesとonline表示は継続する。
            self._cache_available = False

    @property
    def source_summary(self):
        sources = set(self._sources.values())
        if 'online' in sources:
            return '地図: ONLINE'
        if sources.intersection({'cache', 'offline'}):
            return '地図: CACHE / OFFLINE'
        if self._active or self._pending:
            return '地図: tile取得中'
        if self._errors:
            return '地図: オフライン / tileなし'
        return '地図: WGS84位置待受'

    def tile(self, zoom, x, y):
        """tile imageを返し、cache miss時は非同期requestをqueueする。"""
        size = 1 << int(zoom)
        if y < 0 or y >= size:
            return None
        key = (int(zoom), int(x) % size, int(y))
        if key in self._images:
            return self._images[key]

        offline_path = self._path(self.offline_directory, key) if self.offline_directory else ''
        if offline_path and os.path.isfile(offline_path):
            image = self._read_image(offline_path)
            if image is not None:
                self._images[key] = image
                self._sources[key] = 'offline'
                return image

        cached_path = self._path(self.cache_directory, key)
        cached_image = self._read_image(cached_path)
        metadata = self._read_metadata(cached_path) if cached_image is not None else {}
        if cached_image is not None:
            self._images[key] = cached_image
            is_fresh = float(metadata.get('expires_at', 0.0)) > time.time()
            self._sources[key] = 'cache' if is_fresh else 'offline'
            if not is_fresh and self.online_enabled:
                self._queue_request(key, cached_path, metadata)
            return cached_image

        if self.online_enabled:
            self._queue_request(key, cached_path, {})
        else:
            self._errors.add(key)
        return None

    def set_visible_tiles(self, keys):
        """viewport外tileのqueueを捨て、現在表示中だけをrequest対象にする。"""
        self._visible = set(keys)
        self._pending = [entry for entry in self._pending if entry[0] in self._visible]
        self._images = {key: image for key, image in self._images.items()
                        if key in self._visible}
        self._sources = {key: source for key, source in self._sources.items()
                         if key in self._visible}
        self._errors.intersection_update(self._visible)
        self._retry_after = {key: value for key, value in self._retry_after.items()
                             if key in self._visible}
        for reply, (key, _path, _metadata) in list(self._active.items()):
            if key not in self._visible:
                reply.abort()

    @staticmethod
    def _path(root, key):
        zoom, x, y = key
        return os.path.join(root, str(zoom), str(x), '{}.png'.format(y))

    @staticmethod
    def _read_image(path):
        if not path or not os.path.isfile(path):
            return None
        image = QImage(path)
        return image if not image.isNull() else None

    @staticmethod
    def _read_metadata(image_path):
        try:
            with open(image_path + '.json', 'r', encoding='utf-8') as stream:
                return json.load(stream)
        except (OSError, ValueError):
            # 旧cacheにmetadataがない場合も7日間は再利用し、その後は再検証する。
            try:
                return {'expires_at': os.path.getmtime(image_path) + MapTileProvider.FALLBACK_TTL_SEC}
            except OSError:
                return {}

    def _queue_request(self, key, cached_path, metadata):
        if key in self._active or any(entry[0] == key for entry in self._pending):
            return
        if time.monotonic() < self._retry_after.get(key, 0.0):
            return
        self._errors.discard(key)
        self._pending.append((key, cached_path, metadata))
        self._start_requests()

    def _start_requests(self):
        # OSM tile policyに合わせ、同時tile downloadは2件までに制限する。
        while self._pending and len(self._active) < 2:
            key, cached_path, metadata = self._pending.pop(0)
            url = self.url_template.format(z=key[0], x=key[1], y=key[2])
            request = QNetworkRequest(QUrl(url))
            request.setRawHeader(b'User-Agent', self.user_agent.encode('utf-8'))
            if metadata.get('etag'):
                request.setRawHeader(b'If-None-Match', str(metadata['etag']).encode('latin-1'))
            if metadata.get('last_modified'):
                request.setRawHeader(
                    b'If-Modified-Since', str(metadata['last_modified']).encode('latin-1'))
            reply = self._network.get(request)
            self._active[reply] = (key, cached_path, metadata)

    def _request_finished(self, reply):
        item = self._active.pop(reply, None)
        if item is None:
            reply.deleteLater()
            return
        key, cached_path, previous = item
        if key not in self._visible:
            reply.deleteLater()
            self._start_requests()
            return
        status = reply.attribute(QNetworkRequest.HttpStatusCodeAttribute)
        status = int(status) if status is not None else 0
        if status == 304 and os.path.isfile(cached_path):
            metadata = self._metadata_from_headers(reply, previous)
            self._write_metadata(cached_path, metadata)
            self._sources[key] = 'cache'
            self._errors.discard(key)
            self._retry_after.pop(key, None)
        elif reply.error() == QNetworkReply.NoError:
            image = QImage()
            payload = bytes(reply.readAll())
            if image.loadFromData(payload):
                cache_control = bytes(reply.rawHeader(b'Cache-Control')).decode(
                    'latin-1', errors='ignore').lower()
                metadata = self._metadata_from_headers(reply, {})
                if self._cache_available and 'no-store' not in cache_control:
                    self._write_tile(cached_path, payload, metadata)
                self._images[key] = image
                self._sources[key] = 'online'
                self._errors.discard(key)
                self._retry_after.pop(key, None)
            else:
                self._use_stale_cache(key, cached_path)
        elif reply.error() == QNetworkReply.OperationCanceledError:
            # viewport変更で不要になったrequestは通信障害として扱わず、再表示時に取得する。
            self._errors.discard(key)
            self._retry_after.pop(key, None)
        else:
            self._use_stale_cache(key, cached_path)
        reply.deleteLater()
        self.tiles_changed.emit()
        self._start_requests()

    def _use_stale_cache(self, key, cached_path):
        self._errors.add(key)
        self._retry_after[key] = time.monotonic() + 60.0
        image = self._read_image(cached_path)
        if image is None:
            return
        self._images[key] = image
        self._sources[key] = 'offline'

    def _metadata_from_headers(self, reply, previous):
        headers = reply.rawHeaderPairs()
        normalized = {bytes(name).lower(): bytes(value).decode('latin-1')
                      for name, value in headers}
        now = time.time()
        cache_control = normalized.get(b'cache-control', '').lower()
        max_age = re.search(r'(?:^|,)\s*max-age\s*=\s*(\d+)', cache_control)
        if max_age:
            expires_at = now + int(max_age.group(1))
        elif normalized.get(b'expires'):
            try:
                expires_at = email.utils.parsedate_to_datetime(
                    normalized[b'expires']).timestamp()
            except (TypeError, ValueError, OverflowError):
                expires_at = now + self.FALLBACK_TTL_SEC
        else:
            expires_at = now + self.FALLBACK_TTL_SEC
        return {
            'expires_at': expires_at,
            'etag': normalized.get(b'etag', previous.get('etag', '')),
            'last_modified': normalized.get(
                b'last-modified', previous.get('last_modified', '')),
        }

    def _write_tile(self, path, payload, metadata):
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            temp_path = path + '.tmp'
            with open(temp_path, 'wb') as stream:
                stream.write(payload)
            os.replace(temp_path, path)
            self._write_metadata(path, metadata)
        except OSError:
            self._cache_available = False

    @staticmethod
    def _write_metadata(path, metadata):
        try:
            temp_path = path + '.json.tmp'
            with open(temp_path, 'w', encoding='utf-8') as stream:
                json.dump(metadata, stream)
            os.replace(temp_path, path + '.json')
        except OSError:
            pass
