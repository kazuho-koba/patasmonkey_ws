"""confidence分布とRGB depth／ground採用画素の幾何対応を解析する。"""
import csv
from collections import defaultdict
import cv2
import numpy as np


def image_array(message):
    """row paddingとendianを尊重してROS Imageを2D viewへ変換する。"""
    if message.encoding.lower() in ('mono8', '8uc1'):
        dtype = np.dtype('u1')
    elif message.encoding.lower() in ('16uc1', 'mono16'):
        dtype = np.dtype('>u2' if message.is_bigendian else '<u2')
    else:
        raise ValueError('未対応encoding: '+message.encoding)
    if message.step % dtype.itemsize or message.step < message.width*dtype.itemsize:
        raise ValueError('不正なImage.step')
    return np.frombuffer(message.data, dtype=dtype).reshape(
        message.height, message.step//dtype.itemsize)[:, :message.width]


def mapped_confidence(depth_mm, confidence, geometry, rectified_depth=False):
    """RGB depthを3D化し、rectified rightへ投影してconfidenceをnearest参照する。

    SDKの外部translationはcmなのでmへ換算する。K_rightをvirtual Kに使う近似は
    実機未検証。内部warpと同一とは保証しない。対応不成立は-1で、最高信頼0とは
    区別する。confidenceの数値を補間して架空の品質を作らない。
    """
    result = np.full(depth_mm.shape, -1, dtype=np.int16)
    v, u = np.nonzero(depth_mm > 0)
    if len(u) == 0:
        return result
    k = np.asarray(geometry['depth_rgb_K'], dtype=float)
    distortion = None if rectified_depth else np.asarray(geometry['rgb']['D'], dtype=float)
    rays = cv2.undistortPoints(np.column_stack((u, v)).astype(float).reshape(-1, 1, 2),
                               k, distortion).reshape(-1, 2)
    points = np.column_stack((rays, np.ones(len(u))))*(depth_mm[v, u, None]*.001)
    transform = np.asarray(geometry['rgb_to_right_cm'], dtype=float).copy()
    transform[:3, 3] *= .01
    points = points.dot(transform[:3, :3].T)+transform[:3, 3]
    points = points.dot(np.asarray(geometry['right_rectification'], dtype=float).T)
    kr = np.asarray(geometry['right']['K'], dtype=float)
    positive = np.isfinite(points).all(axis=1) & (points[:, 2] > 0)
    x = np.full(len(u), -1, dtype=int); y = x.copy()
    projected = points[positive].dot(kr.T)
    x[positive] = np.rint(projected[:, 0]/projected[:, 2]).astype(int)
    y[positive] = np.rint(projected[:, 1]/projected[:, 2]).astype(int)
    inside = positive & (x >= 0) & (x < confidence.shape[1]) & (y >= 0) & (y < confidence.shape[0])
    result[v[inside], u[inside]] = confidence[y[inside], x[inside]]
    return result


def histogram_summary(histogram):
    """累積binから画素重みのpercentileを求め、巨大な画素配列を再構成しない。"""
    histogram = np.asarray(histogram, dtype=np.int64)
    total = int(histogram.sum())
    result = {'count': total, 'histogram': histogram.tolist()}
    if total:
        result['mean'] = float(np.dot(histogram, np.arange(256))/total)
        for name, quantile in (('median', .5), ('p95', .95), ('p99', .99)):
            result[name] = int(np.searchsorted(histogram.cumsum(), np.ceil(total*quantile)))
        result['maximum'] = int(np.flatnonzero(histogram)[-1])
    return result


def accepted_ground_pixels(path):
    """cells.csvの採用sourceをdeduplicateする。frameの未受理min候補は数えない。

    snapshotに何度も現れる旧採用点はstamp/u/vで一意化する。指定forensic ROIのみの
    分布であり、全mapの全採用ground点を測ったとは解釈しない。
    """
    pixels = defaultdict(set)
    with open(path, newline='') as stream:
        reader = csv.DictReader(stream)
        required = ['last_accepted_ground_input_'+x for x in ('stamp_ns', 'pixel_u', 'pixel_v')]
        if not set(required).issubset(reader.fieldnames or []):
            raise ValueError('ground CSVには'+', '.join(required)+'が必要です')
        for row in reader:
            if not all(row[key] for key in required):
                continue
            stamp, u, v = [int(row[key]) for key in required]
            if stamp > 0 and u >= 0 and v >= 0:
                pixels[stamp].add((u, v))
    return pixels
