#!/usr/bin/env python3
"""Qualify u-blox fixes before navsat_transform consumes them.

The gate does not invent RTK quality: it rejects invalid/stale/implausible
measurements and makes the NavSatFix covariance no more optimistic than the
receiver's NAV-PVT accuracy estimate and a solution-class floor.
"""

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from ublox_msgs.msg import NavPVT


class GnssFixGateNode(Node):
    """Publish only quality-qualified GNSS fixes on a separate topic."""

    def __init__(self) -> None:
        super().__init__("gnss_fix_gate_node")
        self.declare_parameter("input_fix_topic", "/fix")
        self.declare_parameter("navpvt_topic", "/navpvt")
        self.declare_parameter("output_fix_topic", "/fix/gated")
        self.declare_parameter("diagnostics_topic", "/gnss/fix_gate/diagnostics")
        self.declare_parameter("require_navpvt", True)
        self.declare_parameter("navpvt_timeout_sec", 2.5)
        self.declare_parameter("minimum_satellites", 6)
        self.declare_parameter("maximum_horizontal_accuracy_m", 20.0)
        self.declare_parameter("maximum_receiver_speed_mps", 5.0)
        self.declare_parameter("jump_margin_m", 3.0)
        self.declare_parameter("consecutive_fixes_required", 3)
        self.declare_parameter("fixed_covariance_floor_m", 0.5)
        self.declare_parameter("float_covariance_floor_m", 2.0)
        self.declare_parameter("standalone_covariance_floor_m", 10.0)

        value = lambda name: self.get_parameter(name).value
        self.input_fix_topic = str(value("input_fix_topic"))
        self.navpvt_topic = str(value("navpvt_topic"))
        self.output_fix_topic = str(value("output_fix_topic"))
        self.diagnostics_topic = str(value("diagnostics_topic"))
        self.require_navpvt = bool(value("require_navpvt"))
        self.navpvt_timeout_sec = float(value("navpvt_timeout_sec"))
        self.minimum_satellites = int(value("minimum_satellites"))
        self.maximum_horizontal_accuracy_m = float(value("maximum_horizontal_accuracy_m"))
        self.maximum_receiver_speed_mps = float(value("maximum_receiver_speed_mps"))
        self.jump_margin_m = float(value("jump_margin_m"))
        self.consecutive_fixes_required = int(value("consecutive_fixes_required"))
        self.fixed_covariance_floor_m = float(value("fixed_covariance_floor_m"))
        self.float_covariance_floor_m = float(value("float_covariance_floor_m"))
        self.standalone_covariance_floor_m = float(value("standalone_covariance_floor_m"))

        self.latest_navpvt = None
        self.latest_navpvt_received_sec = None
        self.last_candidate = None
        self.last_candidate_stamp_sec = None
        self.consecutive_good = 0
        self.forwarding = False
        self.last_reason = "waiting for GNSS"

        self.publisher = self.create_publisher(NavSatFix, self.output_fix_topic, 10)
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, self.diagnostics_topic, 10
        )
        self.create_subscription(NavPVT, self.navpvt_topic, self.navpvt_callback, 10)
        self.create_subscription(NavSatFix, self.input_fix_topic, self.fix_callback, 10)
        self.publish_status("waiting", self.last_reason)

    @staticmethod
    def stamp_sec(message):
        return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9

    @staticmethod
    def horizontal_distance_m(first, second):
        """Equirectangular distance; accurate enough for a jump gate."""
        earth_radius_m = 6371000.0
        d_lat = math.radians(second.latitude - first.latitude)
        d_lon = math.radians(second.longitude - first.longitude)
        mean_lat = math.radians((first.latitude + second.latitude) / 2.0)
        return earth_radius_m * math.hypot(d_lat, d_lon * math.cos(mean_lat))

    def navpvt_callback(self, message):
        self.latest_navpvt = message
        self.latest_navpvt_received_sec = self.get_clock().now().nanoseconds * 1e-9

    def solution_name(self):
        if self.latest_navpvt is None:
            return "unavailable"
        solution = self.latest_navpvt.flags & NavPVT.FLAGS_CARRIER_PHASE_MASK
        if solution == NavPVT.CARRIER_PHASE_FIXED:
            return "rtk_fixed"
        if solution == NavPVT.CARRIER_PHASE_FLOAT:
            return "rtk_float"
        return "standalone_or_dgnss"

    def covariance_floor_m(self):
        solution = self.solution_name()
        if solution == "rtk_fixed":
            return self.fixed_covariance_floor_m
        if solution == "rtk_float":
            return self.float_covariance_floor_m
        return self.standalone_covariance_floor_m

    def publish_status(self, state, reason):
        status = DiagnosticStatus()
        status.name = "gnss_fix_gate"
        status.hardware_id = "ublox"
        status.level = DiagnosticStatus.OK if self.forwarding else DiagnosticStatus.WARN
        status.message = reason
        navpvt = self.latest_navpvt
        values = [
            KeyValue(key="state", value=state),
            KeyValue(key="solution", value=self.solution_name()),
            KeyValue(key="consecutive_good", value=str(self.consecutive_good)),
            KeyValue(key="required", value=str(self.consecutive_fixes_required)),
        ]
        if navpvt is not None:
            values.extend([
                KeyValue(key="num_sv", value=str(navpvt.num_sv)),
                KeyValue(key="h_acc_m", value="%.3f" % (navpvt.h_acc * 1e-3)),
                KeyValue(key="fix_type", value=str(navpvt.fix_type)),
            ])
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        status.values = values
        self.diagnostics_publisher.publish(array)

    def reject(self, reason):
        if self.forwarding:
            self.get_logger().warn("GNSS fix gate quarantined: %s" % reason)
        self.forwarding = False
        self.consecutive_good = 0
        self.last_candidate = None
        self.last_candidate_stamp_sec = None
        self.last_reason = reason
        self.publish_status("quarantine", reason)

    def navpvt_is_usable(self):
        if self.latest_navpvt is None:
            return False, "NAV-PVT unavailable"
        age = self.get_clock().now().nanoseconds * 1e-9 - self.latest_navpvt_received_sec
        if age > self.navpvt_timeout_sec:
            return False, "NAV-PVT stale (%.2f s)" % age
        navpvt = self.latest_navpvt
        if navpvt.fix_type < NavPVT.FIX_TYPE_3D:
            return False, "NAV-PVT has no 3D fix"
        if not (navpvt.flags & NavPVT.FLAGS_GNSS_FIX_OK):
            return False, "NAV-PVT GNSS_FIX_OK is false"
        if navpvt.num_sv < self.minimum_satellites:
            return False, "only %d satellites" % navpvt.num_sv
        h_acc_m = navpvt.h_acc * 1e-3
        if h_acc_m > self.maximum_horizontal_accuracy_m:
            return False, "h_acc %.2f m exceeds %.2f m" % (
                h_acc_m, self.maximum_horizontal_accuracy_m
            )
        return True, "NAV-PVT valid"

    def fix_callback(self, message):
        if message.status.status < 0:
            self.reject("NavSatFix status is NO_FIX")
            return
        if not (math.isfinite(message.latitude) and math.isfinite(message.longitude)):
            self.reject("NavSatFix latitude/longitude is non-finite")
            return
        if self.require_navpvt:
            valid, reason = self.navpvt_is_usable()
            if not valid:
                self.reject(reason)
                return
        stamp_sec = self.stamp_sec(message)
        if self.last_candidate is not None:
            elapsed = stamp_sec - self.last_candidate_stamp_sec
            if elapsed <= 0.0:
                self.reject("non-monotonic NavSatFix timestamp")
                return
            distance = self.horizontal_distance_m(self.last_candidate, message)
            limit = self.jump_margin_m + self.maximum_receiver_speed_mps * elapsed
            if distance > limit:
                self.reject("position jump %.2f m exceeds %.2f m" % (distance, limit))
                return
        self.last_candidate = message
        self.last_candidate_stamp_sec = stamp_sec
        self.consecutive_good += 1
        if self.consecutive_good < self.consecutive_fixes_required:
            self.forwarding = False
            reason = "warming up: %d/%d valid fixes" % (
                self.consecutive_good, self.consecutive_fixes_required
            )
            self.last_reason = reason
            self.publish_status("warming_up", reason)
            return

        output = NavSatFix()
        output.header = message.header
        output.status = message.status
        output.latitude = message.latitude
        output.longitude = message.longitude
        output.altitude = message.altitude
        output.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        original = message.position_covariance
        reported_sigma_m = 0.0
        if len(original) >= 5 and original[0] >= 0.0 and original[4] >= 0.0:
            reported_sigma_m = math.sqrt(max(original[0], original[4]))
        navpvt_sigma_m = self.latest_navpvt.h_acc * 1e-3 if self.latest_navpvt else 0.0
        sigma_m = max(self.covariance_floor_m(), reported_sigma_m, navpvt_sigma_m)
        output.position_covariance = [0.0] * 9
        output.position_covariance[0] = sigma_m * sigma_m
        output.position_covariance[4] = sigma_m * sigma_m
        # z is not fused in step 4, but retain the incoming vertical covariance.
        output.position_covariance[8] = max(0.0, original[8]) if len(original) >= 9 else 0.0
        self.forwarding = True
        self.last_reason = "forwarding qualified GNSS fix"
        self.publisher.publish(output)
        self.publish_status("forwarding", self.last_reason)


def main(args=None):
    rclpy.init(args=args)
    node = GnssFixGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
