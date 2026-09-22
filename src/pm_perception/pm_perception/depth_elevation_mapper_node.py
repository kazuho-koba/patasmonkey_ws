"""Timestamp-correct depth-to-rolling-elevation mapper for ROS 2 Foxy."""

from collections import deque
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener

from pm_perception.depth_projection import sampled_points, transform_points
from pm_perception.rolling_elevation_grid import RollingElevationGrid


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class DepthElevationMapper(Node):
    """Fuse sampled depth directly into a fixed-allocation 2.5D grid."""

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
            "debug_obstacle_height_max": 0.50,
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
        self.performance_log_period = float(value("performance_log_period"))

        self.grid = RollingElevationGrid(
            float(value("map_size_x")), float(value("map_size_y")),
            float(value("resolution"))
        )
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
        self.last_performance_log = time.monotonic()
        self.dropped_tf = 0
        self.dropped_rate = 0
        self.warned_fallback = False

        self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, 1
        )
        self.create_subscription(
            Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data
        )
        self.retry_timer = self.create_timer(0.01, self.process_pending)
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
        self.get_logger().info(
            "depth-to-elevation mapper ready; all TF lookups use depth stamps; "
            "nominal camera height above ground=%.2f m (Stage 2 reference)"
            % self.nominal_camera_height_above_ground
        )

    def camera_info_callback(self, message):
        if message.k[0] > 0.0 and message.k[4] > 0.0:
            self.camera_info = message

    def depth_callback(self, message):
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
        translation = base_tf.transform.translation
        self.grid.recenter(translation.x, translation.y)
        if points_camera.size:
            points_map = transform_points(points_camera, camera_tf.transform)
            # OAK axial depth is converted to Euclidean optical-frame range.
            # The simple a + b*r^2 model is intentionally conservative; its
            # purpose is to prevent far samples from dominating a cell, not to
            # claim a calibrated OAK-D noise model.
            ranges = np.linalg.norm(points_camera, axis=1)
            point_variance = (
                self.measurement_variance
                + self.depth_variance_per_meter_sq * ranges * ranges
            )
            # camera_z - nominal height is the expected odom z of level
            # ground at this image stamp.  Subtracting it creates a second,
            # local-height layer without modifying the original odom-z map.
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
            self.get_logger().info(
                "terrain frame %.2f ms mean / %.2f ms max, sampled=%d, cells=%d, "
                "rate_drops=%d, tf_drops=%d"
                % (
                    float(values.mean()), float(values.max()), points_camera.shape[0],
                    observed_cells, self.dropped_rate, self.dropped_tf,
                )
            )
            self.last_performance_log = now

    def publish_debug(self, stamp):
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
        if self.publish_debug_pointcloud:
            self.pointcloud_publisher.publish(
                self.make_pointcloud(stamp, elevation, valid)
            )

    def make_debug_grid(self, stamp, layer, valid, minimum, maximum):
        """Encode one scalar layer as a visual-only OccupancyGrid.

        These are not Nav2 costs: 0--100 is only a linear grayscale encoding,
        and -1 is unobserved.  Keeping this conversion at debug publish rate
        avoids image/message work in the depth fusion hot path.
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

    def make_pointcloud(self, stamp, elevation, valid):
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
