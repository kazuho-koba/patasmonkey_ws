"""timestamp整合したOAK depthからrobot-centric rolling elevation mapを作るnode。

hot pathはdepth画像をsamplingし、固定NumPy arrayの`odom` mapへ直接fusionする。full
PointCloud2は意図的に生成しない。camera-to-odomとbase-to-odomの両transformは画像header
stampでlookupする。debug mapと任意cloudは低rateで生成し、可視化がJetson runtime costを
左右しないようにする。
"""

from collections import deque
import csv
from pathlib import Path
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
            # 空文字列なら診断配列・CSVを作らない。bag forensic専用の明示opt-in。
            "forensic_output_dir": "",
            "forensic_roi_forward_min_m": 0.25,
            "forensic_roi_forward_max_m": 4.5,
            "forensic_roi_half_width_m": 0.60,
            # 走行評価CSVから渡されたmap stamp・絶対cellだけに診断対象を限定する。
            "forensic_targets_csv": "",
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
        self.forensic_output_dir = str(value("forensic_output_dir")).strip()
        self.forensic_roi_forward_min_m = float(value("forensic_roi_forward_min_m"))
        self.forensic_roi_forward_max_m = float(value("forensic_roi_forward_max_m"))
        self.forensic_roi_half_width_m = float(value("forensic_roi_half_width_m"))
        self.forensic_targets_csv = str(value("forensic_targets_csv")).strip()
        self.forensic_enabled = bool(self.forensic_output_dir)
        self.forensic_target_cells = set()
        self.forensic_target_spatial_cells = set()
        if self.forensic_targets_csv:
            target_path = Path(self.forensic_targets_csv)
            with target_path.open(encoding="utf-8", newline="") as stream:
                reader = csv.DictReader(stream)
                required = {"map_stamp_ns", "odom_cell_x", "odom_cell_y"}
                if not required.issubset(reader.fieldnames or []):
                    raise ValueError(
                        "forensic target CSV must contain map_stamp_ns, odom_cell_x, odom_cell_y"
                    )
                for row in reader:
                    cell = (int(row["odom_cell_x"]), int(row["odom_cell_y"]))
                    if row["map_stamp_ns"].strip():
                        self.forensic_target_cells.add(
                            (int(row["map_stamp_ns"]), cell[0], cell[1])
                        )
                    else:
                        # 空stampは同じodom-cell座標を全replay時刻で記録する空間target。
                        self.forensic_target_spatial_cells.add(cell)

        self.grid = RollingElevationGrid(
            float(value("map_size_x")), float(value("map_size_y")),
            float(value("resolution")), forensic=self.forensic_enabled,
        )
        self.latest_base_pose = None
        self.forensic_cells_file = None
        self.forensic_support_file = None
        self.forensic_cells_writer = None
        self.forensic_support_writer = None
        if self.forensic_enabled:
            # 出力先を排他的に作成し、過去のdiagnostic CSVを誤って上書きしない。
            output_dir = Path(self.forensic_output_dir)
            output_dir.mkdir(parents=True, exist_ok=True)
            cell_path = output_dir / "hazard_cells.csv"
            support_path = output_dir / "plane_support.csv"
            if cell_path.exists() or support_path.exists():
                raise RuntimeError(
                    "forensic output already exists; choose a fresh directory: {}".format(
                        output_dir
                    )
                )
            self.forensic_cells_file = cell_path.open(
                "x", encoding="utf-8", newline=""
            )
            self.forensic_support_file = support_path.open(
                "x", encoding="utf-8", newline=""
            )
            self.forensic_cells_writer = csv.DictWriter(
                self.forensic_cells_file, fieldnames=self.forensic_cell_fields()
            )
            self.forensic_support_writer = csv.DictWriter(
                self.forensic_support_file, fieldnames=self.forensic_support_fields()
            )
            self.forensic_cells_writer.writeheader()
            self.forensic_support_writer.writeheader()
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
        sampled = sampled_points(
            message, fx, fy, cx, cy, self.pixel_stride,
            self.min_depth, self.max_depth,
            return_pixels=self.forensic_enabled,
        )
        if self.forensic_enabled:
            points_camera, pixel_u, pixel_v, axial_depth = sampled
        else:
            points_camera = sampled
        # base poseはcamera_tfと同じ画像stampから得る。そのXYでrolling windowをrecenterし、
        # yawはStage 3 step cueのsupport方向gateにだけ使う。追加のmap transformではない。
        translation = base_tf.transform.translation
        self.latest_heading_yaw = yaw_from_quaternion(base_tf.transform.rotation)
        if self.forensic_enabled:
            q = base_tf.transform.rotation
            base_roll = float(np.arctan2(
                2.0 * (q.w * q.x + q.y * q.z),
                1.0 - 2.0 * (q.x * q.x + q.y * q.y),
            ))
            base_pitch = float(np.arcsin(np.clip(
                2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0,
            )))
            self.latest_base_pose = (
                translation.x, translation.y, translation.z,
                base_roll, base_pitch, self.latest_heading_yaw,
            )
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
                pixel_u=pixel_u if self.forensic_enabled else None,
                pixel_v=pixel_v if self.forensic_enabled else None,
                axial_depth=axial_depth if self.forensic_enabled else None,
                source_pose=self.latest_base_pose if self.forensic_enabled else None,
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

    @staticmethod
    def forensic_cell_fields():
        """hazard cellごとに一行保存するCSV列を返す。"""
        return [
            "map_stamp_ns", "cell_i", "cell_j", "odom_cell_x", "odom_cell_y",
            "cell_center_x_m", "cell_center_y_m", "forward_m", "lateral_m",
            "hazard", "max_cause", "slope_deg", "roughness_m", "step_m",
            "obstacle_m", "plane_support_count", "step_front_support",
            "step_rear_support", "plane_a", "plane_b", "plane_c",
            "residual_rms_m", "residual_min_m", "residual_max_m",
            "cumulative_observation_count", "cell_age_s", "latest_source_stamp_ns",
            "latest_source_age_s", "frame_sample_count", "frame_min_world_z_m",
            "frame_max_world_z_m", "frame_min_pixel_u", "frame_min_pixel_v",
            "frame_min_axial_depth_m", "frame_max_pixel_u", "frame_max_pixel_v",
            "frame_max_axial_depth_m", "ground_before_fusion_m",
            "ground_after_fusion_m", "relative_ground_before_m",
            "relative_ground_after_m", "fusion_mode", "capture_base_x_m",
            "capture_base_y_m", "capture_base_z_m", "capture_roll_deg",
            "capture_pitch_deg", "capture_yaw_deg",
            "previous_ground_input_stamp_ns", "previous_ground_input_world_z_m",
            "previous_ground_input_relative_z_m", "previous_ground_input_pixel_u",
            "previous_ground_input_pixel_v", "previous_ground_input_depth_m",
            "previous_ground_input_base_z_m", "previous_ground_input_roll_deg",
            "previous_ground_input_pitch_deg", "previous_ground_input_yaw_deg",
            "last_accepted_ground_input_stamp_ns", "last_accepted_ground_input_world_z_m",
            "last_accepted_ground_input_relative_z_m", "last_accepted_ground_input_pixel_u",
            "last_accepted_ground_input_pixel_v", "last_accepted_ground_input_depth_m",
            "last_accepted_ground_input_base_z_m", "last_accepted_ground_input_roll_deg",
            "last_accepted_ground_input_pitch_deg", "last_accepted_ground_input_yaw_deg",
        ]

    @staticmethod
    def forensic_support_fields():
        """平面fit支持点ごとの生値・残差を保存するCSV列を返す。"""
        return [
            "map_stamp_ns", "target_cell_x", "target_cell_y", "target_hazard",
            "target_max_cause", "support_cell_x", "support_cell_y", "dx_cells",
            "dy_cells", "relative_elevation_m", "age_s", "plane_residual_m",
            "frame_sample_count", "frame_min_world_z_m", "frame_max_world_z_m",
            "frame_min_pixel_u", "frame_min_pixel_v", "frame_min_axial_depth_m",
            "frame_max_pixel_u", "frame_max_pixel_v", "frame_max_axial_depth_m",
            "ground_before_fusion_m", "ground_after_fusion_m", "fusion_mode",
            "latest_source_stamp_ns", "capture_base_x_m", "capture_base_y_m",
            "capture_base_z_m", "capture_roll_deg", "capture_pitch_deg",
            "capture_yaw_deg",
        ]

    @staticmethod
    def _csv_value(value):
        """NaN/Infやunknown sentinelを空欄にし、numpy scalarをCSV互換値にする。"""
        if isinstance(value, (float, np.floating)):
            return float(value) if np.isfinite(value) else ""
        if isinstance(value, (int, np.integer)):
            return int(value)
        return value

    def write_forensic_diagnostics(self, stamp, layers, features):
        """現在の黒hazardと平面supportだけをpixel由来までたどって追記する。

        この処理は`forensic_output_dir`指定時だけ呼ばれる。ROIはロボット前方4.5 m以内、
        左右0.60 mに限定し、地図全域の大量な点・ROS message生成を避ける。
        """
        if not self.forensic_enabled or self.latest_base_pose is None:
            return
        hazard = features["hazard"]
        if self.forensic_target_spatial_cells:
            # 追跡対象の同じ絶対odom-cellを時系列に見るmode。hazardの有無に関係なく
            # 観測済みcellを出し、黒になる直前のground fusion履歴も残す。
            abs_x = self.grid.origin_cell_x + np.arange(self.grid.width, dtype=np.int64)
            abs_y = self.grid.origin_cell_y + np.arange(self.grid.height, dtype=np.int64)
            world_x, world_y = np.meshgrid(abs_x, abs_y)
            target_mask = np.zeros((self.grid.height, self.grid.width), dtype=np.bool_)
            for target_x, target_y in self.forensic_target_spatial_cells:
                local_x = target_x - self.grid.origin_cell_x
                local_y = target_y - self.grid.origin_cell_y
                slot = (np.mod(target_y, self.grid.height) * self.grid.width
                        + np.mod(target_x, self.grid.width))
                if (0 <= local_x < self.grid.width and 0 <= local_y < self.grid.height
                        and self.grid.world_x[slot] == target_x
                        and self.grid.world_y[slot] == target_y
                        and self.grid.observation_count[slot] > 0):
                    target_mask[local_y, local_x] = True
            rows, cols = np.nonzero(target_mask)
        else:
            rows, cols = np.nonzero(np.isfinite(hazard) & (hazard >= 1.0))
        if rows.size == 0:
            return

        base_x, base_y, _base_z, base_roll, base_pitch, base_yaw = self.latest_base_pose
        center_x = self.grid.origin_x + (cols + 0.5) * self.grid.resolution
        center_y = self.grid.origin_y + (rows + 0.5) * self.grid.resolution
        dx = center_x - base_x
        dy = center_y - base_y
        forward = dx * np.cos(base_yaw) + dy * np.sin(base_yaw)
        lateral = -dx * np.sin(base_yaw) + dy * np.cos(base_yaw)
        if self.forensic_target_cells or self.forensic_target_spatial_cells:
            # 2段階replayでは経路評価が選んだ正確なmap stampとabsolute cellのみを残す。
            absolute_x = self.grid.origin_cell_x + cols
            absolute_y = self.grid.origin_cell_y + rows
            if self.forensic_target_spatial_cells:
                # 時刻stampのないtargetは危険セルでなくても全観測時点で書く。
                selected = np.ones(rows.size, dtype=np.bool_)
            else:
                selected = np.fromiter(
                    ((stamp_to_ns(stamp), int(cell_x), int(cell_y))
                     in self.forensic_target_cells
                     for cell_x, cell_y in zip(absolute_x, absolute_y)),
                    dtype=np.bool_, count=rows.size,
                )
        else:
            selected = (
                (forward >= self.forensic_roi_forward_min_m)
                & (forward <= self.forensic_roi_forward_max_m)
                & (np.abs(lateral) <= self.forensic_roi_half_width_m)
            )
        rows, cols = rows[selected], cols[selected]
        center_x, center_y = center_x[selected], center_y[selected]
        forward, lateral = forward[selected], lateral[selected]
        if rows.size == 0:
            return

        source = self.grid.forensic_layers()
        stamp_ns = stamp_to_ns(stamp)
        resolution = self.grid.resolution
        radius = max(1, self.feature_neighborhood_radius_cells)
        relative = layers["relative_elevation"]
        age = layers["age_seconds"]
        fresh = (np.isfinite(relative) & np.isfinite(age)
                 & (age <= self.feature_max_observation_age))
        origin_cell_x = self.grid.origin_cell_x
        origin_cell_y = self.grid.origin_cell_y

        for row, col, world_x, world_y, fwd, lat in zip(
                rows, cols, center_x, center_y, forward, lateral):
            cell_x = origin_cell_x + int(col)
            cell_y = origin_cell_y + int(row)
            src = {name: values[row, col] for name, values in source.items()}
            source_stamp = int(src["stamp_ns"])
            cell_row = {
                "map_stamp_ns": stamp_ns,
                "cell_i": int(col), "cell_j": int(row),
                "odom_cell_x": cell_x, "odom_cell_y": cell_y,
                "cell_center_x_m": float(world_x), "cell_center_y_m": float(world_y),
                "forward_m": float(fwd), "lateral_m": float(lat),
                "hazard": self._csv_value(hazard[row, col]),
                "max_cause": int(features["max_cause"][row, col]),
                "slope_deg": self._csv_value(features["slope_deg"][row, col]),
                "roughness_m": self._csv_value(features["roughness"][row, col]),
                "step_m": self._csv_value(features["step_height"][row, col]),
                "obstacle_m": self._csv_value(layers["obstacle_height"][row, col]),
                "plane_support_count": int(features["support_count"][row, col]),
                "step_front_support": int(features["step_support_forward"][row, col]),
                "step_rear_support": int(features["step_support_rear"][row, col]),
                "plane_a": self._csv_value(features["plane_a"][row, col]),
                "plane_b": self._csv_value(features["plane_b"][row, col]),
                "plane_c": self._csv_value(features["plane_c"][row, col]),
                "residual_rms_m": self._csv_value(features["residual_rms"][row, col]),
                "residual_min_m": self._csv_value(features["residual_min"][row, col]),
                "residual_max_m": self._csv_value(features["residual_max"][row, col]),
                "cumulative_observation_count": int(layers["count"][row, col]),
                "cell_age_s": self._csv_value(age[row, col]),
                "latest_source_stamp_ns": source_stamp,
                "latest_source_age_s": max(0.0, (stamp_ns - source_stamp) / 1e9),
                "frame_sample_count": int(src["sample_count"]),
                "frame_min_world_z_m": self._csv_value(src["min_world_z"]),
                "frame_max_world_z_m": self._csv_value(src["max_world_z"]),
                "frame_min_pixel_u": self._csv_value(src["min_pixel_u"]),
                "frame_min_pixel_v": self._csv_value(src["min_pixel_v"]),
                "frame_min_axial_depth_m": self._csv_value(src["min_depth_m"]),
                "frame_max_pixel_u": self._csv_value(src["max_pixel_u"]),
                "frame_max_pixel_v": self._csv_value(src["max_pixel_v"]),
                "frame_max_axial_depth_m": self._csv_value(src["max_depth_m"]),
                "ground_before_fusion_m": self._csv_value(src["ground_before"]),
                "ground_after_fusion_m": self._csv_value(src["ground_after"]),
                "relative_ground_before_m": self._csv_value(src["relative_before"]),
                "relative_ground_after_m": self._csv_value(src["relative_after"]),
                "fusion_mode": int(src["fusion_mode"]),
                "capture_base_x_m": float(src["base_x"]),
                "capture_base_y_m": float(src["base_y"]),
                "capture_base_z_m": float(src["base_z"]),
                "capture_roll_deg": float(np.degrees(src["base_roll"])),
                "capture_pitch_deg": float(np.degrees(src["base_pitch"])),
                "capture_yaw_deg": float(np.degrees(src["base_yaw"])),
            }
            # fusion前に保存しておいた旧accepted sampleと、fusion後の最新accepted inputを
            # 出す。mode 0なら最新accepted inputは以前のままなので、現在frame candidateと
            # 比較して棄却理由を追いやすい。平均値そのものの全入力履歴ではない点に注意する。
            for prefix, output_prefix in (
                    ("previous_ground_input_", "previous_ground_input_"),
                    ("ground_input_", "last_accepted_ground_input_")):
                cell_row[output_prefix + "stamp_ns"] = self._csv_value(
                    src[prefix + "stamp_ns"])
                cell_row[output_prefix + "world_z_m"] = self._csv_value(
                    src[prefix + "world_z"])
                cell_row[output_prefix + "relative_z_m"] = self._csv_value(
                    src[prefix + "relative_z"])
                cell_row[output_prefix + "pixel_u"] = self._csv_value(
                    src[prefix + "pixel_u"])
                cell_row[output_prefix + "pixel_v"] = self._csv_value(
                    src[prefix + "pixel_v"])
                cell_row[output_prefix + "depth_m"] = self._csv_value(
                    src[prefix + "depth_m"])
                cell_row[output_prefix + "base_z_m"] = self._csv_value(
                    src[prefix + "base_z"])
                cell_row[output_prefix + "roll_deg"] = self._csv_value(
                    np.degrees(src[prefix + "base_roll"]))
                cell_row[output_prefix + "pitch_deg"] = self._csv_value(
                    np.degrees(src[prefix + "base_pitch"]))
                cell_row[output_prefix + "yaw_deg"] = self._csv_value(
                    np.degrees(src[prefix + "base_yaw"]))
                if cell_row[output_prefix + "stamp_ns"] == 0:
                    cell_row[output_prefix + "stamp_ns"] = ""
                for pixel_column in ("pixel_u", "pixel_v"):
                    if cell_row[output_prefix + pixel_column] == -1:
                        cell_row[output_prefix + pixel_column] = ""
            self.forensic_cells_writer.writerow(cell_row)

            # 目標cellの平面を作った「同じ時点でfreshな近傍」だけを書き、各残差を再計算。
            a = features["plane_a"][row, col]
            b = features["plane_b"][row, col]
            c = features["plane_c"][row, col]
            for support_row in range(max(0, int(row) - radius),
                                     min(self.grid.height, int(row) + radius + 1)):
                for support_col in range(max(0, int(col) - radius),
                                         min(self.grid.width, int(col) + radius + 1)):
                    if not fresh[support_row, support_col]:
                        continue
                    sx = origin_cell_x + support_col
                    sy = origin_cell_y + support_row
                    offset_x = support_col - int(col)
                    offset_y = support_row - int(row)
                    residual = ""
                    if np.isfinite(a) and np.isfinite(b) and np.isfinite(c):
                        residual = float(
                            relative[support_row, support_col]
                            - (a * offset_x * resolution
                               + b * offset_y * resolution + c)
                        )
                    neighbor = {name: values[support_row, support_col]
                                for name, values in source.items()}
                    self.forensic_support_writer.writerow({
                        "map_stamp_ns": stamp_ns,
                        "target_cell_x": cell_x, "target_cell_y": cell_y,
                        "target_hazard": float(hazard[row, col]),
                        "target_max_cause": int(features["max_cause"][row, col]),
                        "support_cell_x": sx, "support_cell_y": sy,
                        "dx_cells": offset_x, "dy_cells": offset_y,
                        "relative_elevation_m": float(relative[support_row, support_col]),
                        "age_s": float(age[support_row, support_col]),
                        "plane_residual_m": residual,
                        "frame_sample_count": int(neighbor["sample_count"]),
                        "frame_min_world_z_m": self._csv_value(neighbor["min_world_z"]),
                        "frame_max_world_z_m": self._csv_value(neighbor["max_world_z"]),
                        "frame_min_pixel_u": self._csv_value(neighbor["min_pixel_u"]),
                        "frame_min_pixel_v": self._csv_value(neighbor["min_pixel_v"]),
                        "frame_min_axial_depth_m": self._csv_value(neighbor["min_depth_m"]),
                        "frame_max_pixel_u": self._csv_value(neighbor["max_pixel_u"]),
                        "frame_max_pixel_v": self._csv_value(neighbor["max_pixel_v"]),
                        "frame_max_axial_depth_m": self._csv_value(neighbor["max_depth_m"]),
                        "ground_before_fusion_m": self._csv_value(neighbor["ground_before"]),
                        "ground_after_fusion_m": self._csv_value(neighbor["ground_after"]),
                        "fusion_mode": int(neighbor["fusion_mode"]),
                        "latest_source_stamp_ns": int(neighbor["stamp_ns"]),
                        "capture_base_x_m": self._csv_value(neighbor["base_x"]),
                        "capture_base_y_m": self._csv_value(neighbor["base_y"]),
                        "capture_base_z_m": self._csv_value(neighbor["base_z"]),
                        "capture_roll_deg": self._csv_value(
                            np.degrees(neighbor["base_roll"])),
                        "capture_pitch_deg": self._csv_value(
                            np.degrees(neighbor["base_pitch"])),
                        "capture_yaw_deg": self._csv_value(
                            np.degrees(neighbor["base_yaw"])),
                    })

        # 2 Hzのpublish単位でflushし、bag再生が中断しても診断結果を残す。
        self.forensic_cells_file.flush()
        self.forensic_support_file.flush()

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
                    include_diagnostics=self.forensic_enabled,
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
                if self.forensic_enabled:
                    self.write_forensic_diagnostics(stamp, layers, features)
        if self.publish_debug_pointcloud:
            self.pointcloud_publisher.publish(
                self.make_pointcloud(stamp, elevation, valid)
            )

    def destroy_node(self):
        """終了時にforensic CSVを閉じ、最後のbufferも確実にflushする。"""
        for stream in (self.forensic_cells_file, self.forensic_support_file):
            if stream is not None and not stream.closed:
                stream.flush()
                stream.close()
        return super().destroy_node()

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
