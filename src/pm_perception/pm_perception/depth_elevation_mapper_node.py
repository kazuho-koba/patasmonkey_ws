"""timestamp整合したOAK depthからrobot-centric rolling elevation mapを作るnode。

hot pathはdepth画像をsamplingし、固定NumPy arrayの`odom` mapへ直接fusionする。full
PointCloud2は意図的に生成しない。camera-to-odomとbase-to-odomの両transformは画像header
stampでlookupする。debug mapと任意cloudは低rateで生成し、可視化がJetson runtime costを
左右しないようにする。
"""

from collections import deque
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

from pm_perception.depth_projection import sampled_points, transform_points
from pm_perception.rolling_elevation_grid import RollingElevationGrid
from pm_perception.terrain_features import compute_terrain_features


def stamp_to_ns(stamp):
    """ROS header stampをfloating-point誤差なしに整数nsへ変換する。"""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def yaw_from_quaternion(quaternion):
    """TF geometry helperを使わず、平面robot headingを返す。"""
    return float(np.arctan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    ))


class DepthElevationMapper(Node):
    """samplingしたdepthを固定allocationの`odom` 2.5D gridへ直接fusionする。

    `elevation`はodom-z layerのままとする。`relative_elevation`は暫定camera対地高さで
    正規化した別の診断layerであり、globalに一貫したheight mapとして扱ってはならない。
    """

    def __init__(self):
        super().__init__("depth_elevation_mapper")
        defaults = {
            "depth_topic": "/oak/depth/image_raw",
            "camera_info_topic": "/oak/depth/camera_info",
            "map_frame": "odom",
            "base_frame": "base_link",
            "camera_frame_override": "",
            "map_size_x": 6.0,
            "map_size_y": 6.0,
            "resolution": 0.05,
            "nominal_camera_height_above_ground": 0.40,
            "min_depth": 0.4,
            "max_depth": 4.0,
            "pixel_stride": 4,
            "max_processing_rate": 12.0,
            "pending_queue_size": 5,
            "tf_wait_timeout": 0.25,
            "ground_merge_threshold": 0.20,
            "obstacle_min_height": 0.20,
            "measurement_variance": 0.0025,
            "depth_variance_per_meter_sq": 0.0004,
            "observation_decay_time": 8.0,
            "obstacle_confidence_min": 0.15,
            "allow_fallback_intrinsics": True,
            "fallback_fx": 574.28826904,
            "fallback_fy": 574.28826904,
            "fallback_cx": 354.75085449,
            "fallback_cy": 215.23262024,
            "fallback_width": 640,
            "fallback_height": 400,
            "publish_debug_occupancy": True,
            "publish_debug_pointcloud": False,
            "debug_publish_rate": 2.0,
            "debug_elevation_min": -0.5,
            "debug_elevation_max": 0.5,
            "publish_stage2_debug_layers": True,
            "debug_relative_elevation_min": -0.30,
            "debug_relative_elevation_max": 0.30,
            "debug_variance_max": 0.02,
            "debug_count_saturation": 10,
            "debug_age_max": 5.0,
            "debug_obstacle_height_max": 0.20,
            "publish_stage3_debug_layers": True,
            "feature_max_observation_age": 3.0,
            "feature_neighborhood_radius_cells": 1,
            "feature_min_neighbors": 5,
            "step_min_side_neighbors": 2,
            "hazard_slope_limit_deg": 20.0,
            "hazard_roughness_limit": 0.03,
            "hazard_step_limit": 0.07,
            "hazard_obstacle_height_limit": 0.20,
            "debug_slope_max_deg": 20.0,
            "debug_roughness_max": 0.03,
            "debug_step_height_max": 0.07,
            "publish_hazard_cause_markers": False,
            "hazard_marker_max_points": 2500,
            "hazard_marker_z_offset": 0.04,
            "performance_log_period": 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def value(name):
            return self.get_parameter(name).value

        self.depth_topic = str(value("depth_topic"))
        self.camera_info_topic = str(value("camera_info_topic"))
        self.map_frame = str(value("map_frame"))
        self.base_frame = str(value("base_frame"))
        self.camera_frame_override = str(value("camera_frame_override"))
        self.min_depth = float(value("min_depth"))
        self.max_depth = float(value("max_depth"))
        self.nominal_camera_height_above_ground = float(
            value("nominal_camera_height_above_ground")
        )
        self.pixel_stride = int(value("pixel_stride"))
        self.minimum_period_ns = int(
            1e9 / max(float(value("max_processing_rate")), 0.001)
        )
        self.pending_queue_size = int(value("pending_queue_size"))
        self.tf_wait_timeout = float(value("tf_wait_timeout"))
        self.ground_merge_threshold = float(value("ground_merge_threshold"))
        self.obstacle_min_height = float(value("obstacle_min_height"))
        self.measurement_variance = float(value("measurement_variance"))
        self.depth_variance_per_meter_sq = float(
            value("depth_variance_per_meter_sq")
        )
        self.observation_decay_time = float(value("observation_decay_time"))
        self.obstacle_confidence_min = float(value("obstacle_confidence_min"))
        self.allow_fallback_intrinsics = bool(value("allow_fallback_intrinsics"))
        self.fallback = (
            float(value("fallback_fx")), float(value("fallback_fy")),
            float(value("fallback_cx")), float(value("fallback_cy")),
            int(value("fallback_width")), int(value("fallback_height")),
        )
        self.publish_debug_occupancy = bool(value("publish_debug_occupancy"))
        self.publish_debug_pointcloud = bool(value("publish_debug_pointcloud"))
        self.debug_elevation_min = float(value("debug_elevation_min"))
        self.debug_elevation_max = float(value("debug_elevation_max"))
        self.publish_stage2_debug_layers = bool(
            value("publish_stage2_debug_layers")
        )
        self.debug_relative_elevation_min = float(
            value("debug_relative_elevation_min")
        )
        self.debug_relative_elevation_max = float(
            value("debug_relative_elevation_max")
        )
        self.debug_variance_max = float(value("debug_variance_max"))
        self.debug_count_saturation = int(value("debug_count_saturation"))
        self.debug_age_max = float(value("debug_age_max"))
        self.debug_obstacle_height_max = float(
            value("debug_obstacle_height_max")
        )
        self.publish_stage3_debug_layers = bool(
            value("publish_stage3_debug_layers")
        )
        self.feature_max_observation_age = float(
            value("feature_max_observation_age")
        )
        self.feature_neighborhood_radius_cells = int(
            value("feature_neighborhood_radius_cells")
        )
        self.feature_min_neighbors = int(value("feature_min_neighbors"))
        self.step_min_side_neighbors = int(value("step_min_side_neighbors"))
        self.hazard_slope_limit_deg = float(value("hazard_slope_limit_deg"))
        self.hazard_roughness_limit = float(value("hazard_roughness_limit"))
        self.hazard_step_limit = float(value("hazard_step_limit"))
        self.hazard_obstacle_height_limit = float(
            value("hazard_obstacle_height_limit")
        )
        self.debug_slope_max_deg = float(value("debug_slope_max_deg"))
        self.debug_roughness_max = float(value("debug_roughness_max"))
        self.debug_step_height_max = float(value("debug_step_height_max"))
        self.publish_hazard_cause_markers = bool(
            value("publish_hazard_cause_markers")
        )
        self.hazard_marker_max_points = int(value("hazard_marker_max_points"))
        self.hazard_marker_z_offset = float(value("hazard_marker_z_offset"))
        self.performance_log_period = float(value("performance_log_period"))

        self.grid = RollingElevationGrid(
            float(value("map_size_x")), float(value("map_size_y")),
            float(value("resolution"))
        )
        # bounded FIFOはdepth stampに対応するTF/CameraInfoだけを待つ。TF欠落時にもlatencyと
        # memoryを制限できる。ここで最新transformを使うと移動中robotのmapが空間的にずれる。
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pending = deque()
        self.camera_info = None
        self.last_processed_stamp_ns = -1
        self.last_debug_ns = -1
        self.debug_period_ns = int(
            1e9 / max(float(value("debug_publish_rate")), 0.001)
        )
        self.processing_times_ms = deque(maxlen=100)
        self.feature_times_ms = deque(maxlen=100)
        self.last_performance_log = time.monotonic()
        self.dropped_tf = 0
        self.dropped_rate = 0
        self.warned_fallback = False
        self.latest_heading_yaw = 0.0

        self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, 1
        )
        self.create_subscription(
            Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data
        )
        self.retry_timer = self.create_timer(0.01, self.process_pending)
        # 以下のOccupancyGrid publisherはすべて可視化診断用であり、Nav2 costではない。private
        # nameにして、実験中のAPIを既存planner/localisation interfaceから隔離する。
        self.occupancy_publisher = self.create_publisher(
            OccupancyGrid, "~/elevation_debug", 1
        )
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2, "~/elevation_points_debug", 1
        )
        self.relative_elevation_publisher = self.create_publisher(
            OccupancyGrid, "~/relative_elevation_debug", 1
        )
        self.variance_publisher = self.create_publisher(
            OccupancyGrid, "~/elevation_variance_debug", 1
        )
        self.count_publisher = self.create_publisher(
            OccupancyGrid, "~/observation_count_debug", 1
        )
        self.age_publisher = self.create_publisher(
            OccupancyGrid, "~/observation_age_debug", 1
        )
        self.obstacle_publisher = self.create_publisher(
            OccupancyGrid, "~/obstacle_height_debug", 1
        )
        self.slope_publisher = self.create_publisher(
            OccupancyGrid, "~/slope_debug", 1
        )
        self.roughness_publisher = self.create_publisher(
            OccupancyGrid, "~/roughness_debug", 1
        )
        self.step_height_publisher = self.create_publisher(
            OccupancyGrid, "~/step_height_debug", 1
        )
        self.hazard_publisher = self.create_publisher(
            OccupancyGrid, "~/terrain_hazard_debug", 1
        )
        self.hazard_cause_publisher = self.create_publisher(
            OccupancyGrid, "~/terrain_hazard_cause_debug", 1
        )
        self.hazard_cause_marker_publisher = self.create_publisher(
            Marker, "~/terrain_hazard_cause_markers", 1
        )
        self.get_logger().info(
            "depth-to-elevation mapper ready; all TF lookups use depth stamps; "
            "nominal camera height above ground=%.2f m (Stage 2 reference)"
            % self.nominal_camera_height_above_ground
        )

    def camera_info_callback(self, message):
        """使用可能なpinhole calibrationだけを保持し、画像サイズは後で照合する。"""
        if message.k[0] > 0.0 and message.k[4] > 0.0:
            self.camera_info = message

    def depth_callback(self, message):
        """入力をrate制限した後、短いexact-TF待ちqueueへ入れる。

        queue内で最も古い画像を捨てることで、一時的なTF停止後に古いdepthを処理するより現在の
        mapを優先する。受理する各画像のpose/time対応を守れるため、最新TFへの置換よりdropが
        安全である。
        """
        stamp_ns = stamp_to_ns(message.header.stamp)
        if (
            self.last_processed_stamp_ns >= 0
            and stamp_ns - self.last_processed_stamp_ns < self.minimum_period_ns
        ):
            self.dropped_rate += 1
            return
        self.pending.append((time.monotonic(), message))
        while len(self.pending) > self.pending_queue_size:
            self.pending.popleft()
            self.dropped_tf += 1
        self.process_pending()

    def intrinsics_for(self, message):
        """この画像geometryに対応する場合だけcalibrationを返す。

        EEPROM fallbackはCameraInfoを持たない古いbag専用である。別streamへ640x400の
        calibrationを暗黙適用しないよう、設定済みwidth/heightでguardする。
        """
        info = self.camera_info
        if info is not None and info.width == message.width and info.height == message.height:
            return float(info.k[0]), float(info.k[4]), float(info.k[2]), float(info.k[5])
        fx, fy, cx, cy, width, height = self.fallback
        if self.allow_fallback_intrinsics and width == message.width and height == message.height:
            if not self.warned_fallback:
                self.get_logger().warn(
                    "CameraInfo unavailable; using configured EEPROM fallback intrinsics"
                )
                self.warned_fallback = True
            return fx, fy, cx, cy
        return None

    def process_pending(self):
        """exactな入力が揃ったqueue画像をtimestamp順に処理する。

        queue先頭は後続frameを短時間blockする。これによりfusionの時間順を保ち、bag replay中の
        timeout方針も決定的になる。time zero（最新TF）でのlookupは許可しない。
        """
        while self.pending:
            arrival, message = self.pending[0]
            intrinsics = self.intrinsics_for(message)
            if intrinsics is None:
                if time.monotonic() - arrival <= self.tf_wait_timeout:
                    return
                self.pending.popleft()
                self.get_logger().warn("dropping depth frame: no matching CameraInfo")
                continue
            source_frame = self.camera_frame_override or message.header.frame_id
            stamp = Time.from_msg(message.header.stamp)
            try:
                camera_tf = self.tf_buffer.lookup_transform(
                    self.map_frame, source_frame, stamp
                )
                base_tf = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, stamp
                )
            except TransformException as error:
                if time.monotonic() - arrival <= self.tf_wait_timeout:
                    return
                self.pending.popleft()
                self.dropped_tf += 1
                self.get_logger().warn(
                    "dropping depth frame after timestamped TF timeout: " + str(error)
                )
                continue
            self.pending.popleft()
            self.process_frame(message, intrinsics, camera_tf, base_tf)

    def process_frame(self, message, intrinsics, camera_tf, base_tf):
        """1画像をback-projectし、そのstampでtransformしてcellへfusionする。"""
        started = time.perf_counter()
        stamp_ns = stamp_to_ns(message.header.stamp)
        if (
            self.last_processed_stamp_ns >= 0
            and stamp_ns - self.last_processed_stamp_ns < self.minimum_period_ns
        ):
            self.dropped_rate += 1
            return
        self.last_processed_stamp_ns = stamp_ns

        fx, fy, cx, cy = intrinsics
        points_camera = sampled_points(
            message, fx, fy, cx, cy, self.pixel_stride,
            self.min_depth, self.max_depth
        )
        # base poseはcamera_tfと同じ画像stampから得る。そのXYでrolling windowをrecenterし、
        # yawはStage 3 step cueのsupport方向gateにだけ使う。追加のmap transformではない。
        translation = base_tf.transform.translation
        self.latest_heading_yaw = yaw_from_quaternion(base_tf.transform.rotation)
        self.grid.recenter(translation.x, translation.y)
        if points_camera.size:
            points_map = transform_points(points_camera, camera_tf.transform)
            # OAKのaxial depthをEuclidean optical-frame rangeへ変換する。単純な`a+b*r^2`
            # modelは意図的に保守的である。遠方sampleがcellを支配しないようにするもので、
            # 校正済みOAK-D noise modelを主張するものではない。
            ranges = np.linalg.norm(points_camera, axis=1)
            point_variance = (
                self.measurement_variance
                + self.depth_variance_per_meter_sq * ranges * ranges
            )
            # `camera_z - nominal height`は、この画像stampにおける水平groundの期待odom zで
            # ある。これを差し引くと、元のodom-z mapを変更せずにlocal-height layerを作れる。
            relative_offset = (
                self.nominal_camera_height_above_ground
                - camera_tf.transform.translation.z
            )
            observed_cells = self.grid.fuse_points(
                points_map[:, 0], points_map[:, 1], points_map[:, 2], stamp_ns,
                self.ground_merge_threshold, self.obstacle_min_height,
                point_variance, relative_offset, self.observation_decay_time,
            )
        else:
            observed_cells = 0

        # 高価なmessage生成と局所平面featureはdebug処理なので、depth fusion rateとは独立して
        # 実行する。
        if (
            stamp_ns - self.last_debug_ns >= self.debug_period_ns
            or self.last_debug_ns < 0
        ):
            self.publish_debug(message.header.stamp)
            self.last_debug_ns = stamp_ns
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self.processing_times_ms.append(elapsed_ms)
        now = time.monotonic()
        if now - self.last_performance_log >= self.performance_log_period:
            values = np.asarray(self.processing_times_ms)
            feature_values = np.asarray(self.feature_times_ms)
            self.get_logger().info(
                "terrain frame %.2f ms mean / %.2f ms max, feature %.2f ms mean, "
                "sampled=%d, cells=%d, rate_drops=%d, tf_drops=%d"
                % (
                    float(values.mean()), float(values.max()),
                    float(feature_values.mean()) if feature_values.size else 0.0,
                    points_camera.shape[0], observed_cells, self.dropped_rate,
                    self.dropped_tf,
                )
            )
            self.last_performance_log = now

    def publish_debug(self, stamp):
        """一貫したgrid snapshotから低rateの検査layerをpublishする。"""
        layers = self.grid.stage2_layers(
            self.measurement_variance, stamp_to_ns(stamp),
            self.obstacle_confidence_min, self.observation_decay_time,
        )
        elevation = layers["elevation"]
        valid = layers["count"] > 0
        if self.publish_debug_occupancy:
            self.occupancy_publisher.publish(self.make_debug_grid(
                stamp, elevation, valid, self.debug_elevation_min,
                self.debug_elevation_max,
            ))
            if self.publish_stage2_debug_layers:
                self.relative_elevation_publisher.publish(self.make_debug_grid(
                    stamp, layers["relative_elevation"], valid,
                    self.debug_relative_elevation_min,
                    self.debug_relative_elevation_max,
                ))
                self.variance_publisher.publish(self.make_debug_grid(
                    stamp, layers["variance"], valid, 0.0,
                    self.debug_variance_max,
                ))
                self.count_publisher.publish(self.make_debug_grid(
                    stamp, layers["count"].astype(np.float32), valid, 0.0,
                    float(max(1, self.debug_count_saturation)),
                ))
                self.age_publisher.publish(self.make_debug_grid(
                    stamp, layers["age_seconds"], valid, 0.0,
                    self.debug_age_max,
                ))
                obstacle_valid = np.isfinite(layers["obstacle_height"])
                self.obstacle_publisher.publish(self.make_debug_grid(
                    stamp, layers["obstacle_height"], obstacle_valid, 0.0,
                    self.debug_obstacle_height_max,
                ))
            if self.publish_stage3_debug_layers:
                # featureにはrelative elevationを使い、common-mode odom-z offsetがterrain
                # shapeに見えることを抑える。局所平面supportが不足してもobstacle evidenceは
                # 独立に有効であり、compute_terrain_features()がそのように扱う。
                feature_started = time.perf_counter()
                features = compute_terrain_features(
                    layers["relative_elevation"], layers["age_seconds"],
                    self.grid.resolution, self.feature_max_observation_age,
                    self.feature_neighborhood_radius_cells,
                    self.feature_min_neighbors, self.hazard_slope_limit_deg,
                    self.hazard_roughness_limit, self.hazard_step_limit,
                    layers["obstacle_height"],
                    self.hazard_obstacle_height_limit, self.latest_heading_yaw,
                    self.step_min_side_neighbors,
                )
                self.feature_times_ms.append(
                    (time.perf_counter() - feature_started) * 1000.0
                )
                self.slope_publisher.publish(self.make_debug_grid(
                    stamp, features["slope_deg"], np.isfinite(features["slope_deg"]),
                    0.0, self.debug_slope_max_deg,
                ))
                self.roughness_publisher.publish(self.make_debug_grid(
                    stamp, features["roughness"], np.isfinite(features["roughness"]),
                    0.0, self.debug_roughness_max,
                ))
                self.step_height_publisher.publish(self.make_debug_grid(
                    stamp, features["step_height"], np.isfinite(features["step_height"]),
                    0.0, self.debug_step_height_max,
                ))
                self.hazard_publisher.publish(self.make_debug_grid(
                    stamp, features["hazard"], np.isfinite(features["hazard"]),
                    0.0, 1.0,
                ))
                cause_valid = features["max_cause"] > 0
                self.hazard_cause_publisher.publish(self.make_debug_grid(
                    stamp, features["max_cause"].astype(np.float32),
                    cause_valid, 0.0, 4.0,
                ))
                if self.publish_hazard_cause_markers:
                    self.hazard_cause_marker_publisher.publish(
                        self.make_hazard_cause_marker(
                            stamp, elevation, features["hazard"],
                            features["max_cause"],
                        )
                    )
        if self.publish_debug_pointcloud:
            self.pointcloud_publisher.publish(
                self.make_pointcloud(stamp, elevation, valid)
            )

    def make_debug_grid(self, stamp, layer, valid, minimum, maximum):
        """1個のscalar layerを可視化専用OccupancyGridへ符号化する。

        Nav2 costではない。値は0--100へ線形符号化し、-1は未観測である。RVizの`map` paletteは
        通常のOccupancyGrid方向（0白、100黒、-1灰）で描画する。この変換をdebug publish rateに
        留めることで、depth fusion hot pathでのimage/message処理を避ける。
        """
        message = OccupancyGrid()
        message.header.stamp = stamp
        message.header.frame_id = self.map_frame
        message.info.resolution = self.grid.resolution
        message.info.width = self.grid.width
        message.info.height = self.grid.height
        message.info.origin.position.x = self.grid.origin_x
        message.info.origin.position.y = self.grid.origin_y
        message.info.origin.orientation.w = 1.0
        encoded = np.full(layer.shape, -1, dtype=np.int8)
        span = max(maximum - minimum, 1e-6)
        encoded[valid] = np.clip(
            np.rint(100.0 * (layer[valid] - minimum) / span), 0, 100
        ).astype(np.int8)
        message.data = encoded.ravel().tolist()
        return message

    def make_hazard_cause_marker(self, stamp, elevation, hazard, cause):
        """RVizで明示的に有効化したときだけ、上限付き色cubeをpublishする。

        Markerはsaturated（hazard >= 1）cellだけを表す。上限によりPoint/Color messageの無制限
        な増加を防ぐ。これはofflineでの説明補助であり、完全なterrain-map表現ではない。
        """
        message = Marker()
        message.header.stamp = stamp
        message.header.frame_id = self.map_frame
        message.ns = "terrain_hazard_cause"
        message.id = 0
        message.type = Marker.CUBE_LIST
        message.action = Marker.ADD
        message.pose.orientation.w = 1.0
        message.scale.x = self.grid.resolution
        message.scale.y = self.grid.resolution
        message.scale.z = 0.03
        message.lifetime.sec = 1
        rows, cols = np.nonzero((hazard >= 1.0) & (cause > 0))
        if rows.size > max(0, self.hazard_marker_max_points):
            selection = np.linspace(
                0, rows.size - 1, self.hazard_marker_max_points, dtype=np.intp
            )
            rows, cols = rows[selection], cols[selection]
        colors = (
            (1.0, 0.15, 0.15),  # slope: red
            (0.15, 0.85, 0.15),  # roughness: green
            (1.0, 0.8, 0.05),  # forward step: yellow
            (0.9, 0.1, 0.9),  # obstacle: magenta
        )
        for row, col in zip(rows, cols):
            point = Point()
            point.x = self.grid.origin_x + (col + 0.5) * self.grid.resolution
            point.y = self.grid.origin_y + (row + 0.5) * self.grid.resolution
            point.z = elevation[row, col] + self.hazard_marker_z_offset
            message.points.append(point)
            red, green, blue = colors[cause[row, col] - 1]
            message.colors.append(ColorRGBA(r=red, g=green, b=blue, a=0.9))
        return message

    def make_pointcloud(self, stamp, elevation, valid):
        """任意RViz検査用に、有効cellあたりground pointを1個作る。

        ROS PointCloud2のallocateとserializeを行うため既定OFFとし、depth fusionやterrain
        featureでは使用しない。
        """
        rows, cols = np.nonzero(valid)
        points = np.empty((rows.size, 3), dtype=np.float32)
        points[:, 0] = self.grid.origin_x + (cols + 0.5) * self.grid.resolution
        points[:, 1] = self.grid.origin_y + (rows + 0.5) * self.grid.resolution
        points[:, 2] = elevation[rows, cols]
        message = PointCloud2()
        message.header.stamp = stamp
        message.header.frame_id = self.map_frame
        message.height = 1
        message.width = points.shape[0]
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = 12 * points.shape[0]
        message.data = points.tobytes()
        message.is_dense = True
        return message


def main(args=None):
    rclpy.init(args=args)
    node = DepthElevationMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
