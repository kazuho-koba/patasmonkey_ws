"""URDFのvisual meshをbase_link基準の三角形群へ展開する。"""

import math
import os
import struct
import xml.etree.ElementTree as ET

import numpy as np
from ament_index_python.packages import get_package_share_directory


def _origin_matrix(origin):
    """URDF originのxyz [m]と固定軸roll/pitch/yaw [rad]を同次変換にする。"""
    if origin is None:
        xyz = (0.0, 0.0, 0.0)
        rpy = (0.0, 0.0, 0.0)
    else:
        xyz = tuple(float(value) for value in origin.get(
            'xyz', '0 0 0').split())
        rpy = tuple(float(value) for value in origin.get(
            'rpy', '0 0 0').split())
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    matrix[:3, 3] = xyz
    return matrix


def _read_stl(path):
    """binaryまたはASCII STLから各facetの3頂点をメートル単位で読む。"""
    with open(path, 'rb') as stream:
        data = stream.read()
    if len(data) >= 84:
        triangle_count = struct.unpack_from('<I', data, 80)[0]
        if 84 + triangle_count * 50 == len(data):
            triangles = np.empty((triangle_count, 3, 3), dtype=np.float64)
            for index in range(triangle_count):
                values = struct.unpack_from('<12fH', data, 84 + index * 50)
                triangles[index] = np.asarray(values[3:12]).reshape((3, 3))
            return triangles

    # ASCII STLはvertex行を順に3個ずつまとめる。
    vertices = []
    for line in data.decode('ascii', errors='ignore').splitlines():
        fields = line.strip().split()
        if len(fields) == 4 and fields[0].lower() == 'vertex':
            vertices.append(tuple(float(value) for value in fields[1:]))
    if not vertices or len(vertices) % 3:
        raise ValueError('STLのfacetデータを読み取れません: {}'.format(path))
    return np.asarray(vertices, dtype=np.float64).reshape((-1, 3, 3))


class UrdfRobotModel:
    """URDFリンクとvisual meshをたどり、GUI描画用にmesh変換を事前計算する。"""

    COLORS = {
        'body': (30, 92, 48),
        'wheelarm': (238, 240, 232),
        'wheel': (18, 20, 22),
    }

    def __init__(self, urdf_path=None):
        if urdf_path:
            self.urdf_path = os.path.abspath(os.path.expanduser(str(urdf_path)))
        else:
            package_share = get_package_share_directory('pm_description')
            self.urdf_path = os.path.join(package_share, 'urdf', 'pm.urdf')
        self._package_directories = {}
        (self.triangles, self.colors, self.body_front_anchor,
         self.body_center_anchor) = self._load()
        self.normals = self._face_normals(self.triangles)
        self.bounds_center = (self.triangles.reshape((-1, 3)).min(axis=0)
                              + self.triangles.reshape((-1, 3)).max(axis=0)) * 0.5

    @staticmethod
    def _face_normals(triangles):
        edges_a = triangles[:, 1] - triangles[:, 0]
        edges_b = triangles[:, 2] - triangles[:, 0]
        normals = np.cross(edges_a, edges_b)
        lengths = np.linalg.norm(normals, axis=1)
        lengths[lengths < 1e-12] = 1.0
        return normals / lengths[:, None]

    def _resolve_mesh(self, filename):
        if filename.startswith('package://'):
            package_and_path = filename[len('package://'):].split('/', 1)
            package = package_and_path[0]
            if package not in self._package_directories:
                self._package_directories[package] = get_package_share_directory(package)
            return os.path.join(self._package_directories[package], package_and_path[1])
        return os.path.abspath(os.path.expanduser(filename))

    @staticmethod
    def _link_transforms(root):
        """joint originを親から子へ積み上げ、各linkからbase_linkへの姿勢を得る。"""
        parent_joint = {}
        for joint in root.findall('joint'):
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            parent_joint[child] = (parent, _origin_matrix(joint.find('origin')))

        transforms = {'base_link': np.eye(4, dtype=np.float64)}

        def resolve(link, visiting):
            if link in transforms:
                return transforms[link]
            if link in visiting or link not in parent_joint:
                raise ValueError('base_linkから辿れないURDF linkです: {}'.format(link))
            parent, parent_to_child = parent_joint[link]
            result = resolve(parent, visiting | {link}) @ parent_to_child
            transforms[link] = result
            return result

        for link in root.findall('link'):
            resolve(link.get('name'), set())
        return transforms

    def _load(self):
        root = ET.parse(self.urdf_path).getroot()
        transforms = self._link_transforms(root)
        triangle_sets = []
        color_sets = []
        body_vertices = []
        for link in root.findall('link'):
            link_name = link.get('name', '')
            if link_name == 'body':
                color = self.COLORS['body']
            elif link_name.startswith('wheelarm_'):
                color = self.COLORS['wheelarm']
            elif link_name.startswith('wheel_'):
                color = self.COLORS['wheel']
            else:
                continue

            for visual in link.findall('visual'):
                mesh = visual.find('./geometry/mesh')
                if mesh is None:
                    continue
                mesh_path = self._resolve_mesh(mesh.get('filename', ''))
                scale = np.asarray([
                    float(value) for value in mesh.get('scale', '1 1 1').split()
                ], dtype=np.float64)
                mesh_origin = _origin_matrix(visual.find('origin'))
                base_to_visual = transforms[link_name] @ mesh_origin
                triangles = _read_stl(mesh_path) * scale[None, None, :]
                homogeneous = np.concatenate((triangles,
                                              np.ones((*triangles.shape[:2], 1))), axis=2)
                transformed = homogeneous @ base_to_visual.T
                triangle_sets.append(transformed[:, :, :3])
                if link_name == 'body':
                    # シャシmeshの前端近傍をbase_link座標で保存し、矢印の固定点に使う。
                    body_vertices.append(transformed[:, :, :3].reshape((-1, 3)))
                color_sets.append(np.repeat(
                    np.asarray(color, dtype=np.uint8)[None, :], len(triangles), axis=0))

        if not triangle_sets or not body_vertices:
            raise ValueError('URDFから車体またはシャシmeshを読み込めません: {}'.format(
                self.urdf_path))
        body_points = np.concatenate(body_vertices)
        front_x = float(body_points[:, 0].max())
        # STLの三角形頂点密度に引っ張られないよう、前端2%幅のy/z中央値を中央位置にする。
        front_band = max(float(np.ptp(body_points[:, 0])) * 0.02, 0.002)
        front_slice = body_points[body_points[:, 0] >= front_x - front_band]
        body_front_anchor = np.asarray((
            front_x, float(np.median(front_slice[:, 1])),
            float(np.median(front_slice[:, 2])),
        ), dtype=np.float64)
        body_center_anchor = (body_points.min(axis=0) + body_points.max(axis=0)) * 0.5
        return (np.concatenate(triangle_sets), np.concatenate(color_sets),
                body_front_anchor, body_center_anchor)
