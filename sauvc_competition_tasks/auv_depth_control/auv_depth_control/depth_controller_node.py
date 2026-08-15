import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import SetBool


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class DepthController(Node):
    """Pressure-primary depth PID; optional mission Float32 setpoint; optional auto-arm at startup."""

    def __init__(self) -> None:
        super().__init__("depth_controller")

        self.declare_parameter("depth_topic", "/auv/depth")
        self.declare_parameter("ping_topic", "/ping1d/data")
        self.declare_parameter("heave_cmd_topic", "/control/heave_cmd")

        self.declare_parameter("target_depth_m", 0.5)
        self.declare_parameter("kp", 0.35)
        self.declare_parameter("ki", 0.02)
        self.declare_parameter("kd", 0.15)
        self.declare_parameter("integral_limit", 0.4)
        self.declare_parameter("max_heave_cmd", 0.25)
        self.declare_parameter("min_heave_cmd", -0.25)
        self.declare_parameter("output_sign", 1.0)
        self.declare_parameter("heave_pwm_neutral", 1500.0)
        self.declare_parameter("heave_pwm_per_unit", 400.0)

        self.declare_parameter("depth_derivative_lp_alpha", 0.35)
        self.declare_parameter("max_depth_dt_sec", 2.0)

        self.declare_parameter("use_ping_guard", True)
        self.declare_parameter("min_ping_valid_m", 0.12)
        self.declare_parameter("max_ping_valid_m", 8.0)
        self.declare_parameter("min_clearance_ping_m", 0.25)
        self.declare_parameter("ping_guard_gain", 1.5)
        self.declare_parameter("ping_stale_sec", 3.0)

        self.declare_parameter("reset_integral_on_arm", True)
        self.declare_parameter("enable_debug_logs", False)

        self.declare_parameter("mission_target_depth_topic", "")
        # If > 0, use mission Float32 only while age < this (seconds). If <= 0, latch last
        # mission depth until a new message (never revert to target_depth_m while latched).
        self.declare_parameter("mission_target_stale_sec", 0.0)

        self.declare_parameter("auto_arm_on_start", False)

        self._armed = False
        self._mission_target_depth: Optional[float] = None
        self._mission_target_stamp: Optional[Time] = None
        self._integral = 0.0
        self._last_depth: Optional[float] = None
        self._last_depth_time = self.get_clock().now()
        self._d_meas_filt = 0.0
        self._last_ping: Optional[float] = None
        self._last_ping_time = self.get_clock().now()

        depth_topic = str(self.get_parameter("depth_topic").value)
        ping_topic = str(self.get_parameter("ping_topic").value)
        heave_topic = str(self.get_parameter("heave_cmd_topic").value)

        self._heave_pub = self.create_publisher(Vector3, heave_topic, 10)
        self.create_subscription(Float32, depth_topic, self._on_depth, 10)
        self.create_subscription(Float32, ping_topic, self._on_ping, 10)
        self.create_service(SetBool, "arm", self._on_arm)

        mt_topic = str(self.get_parameter("mission_target_depth_topic").value).strip()
        if mt_topic:
            self.create_subscription(Float32, mt_topic, self._on_mission_target_depth, 10)

        if bool(self.get_parameter("auto_arm_on_start").value):
            if bool(self.get_parameter("reset_integral_on_arm").value):
                self._integral = 0.0
                self._d_meas_filt = 0.0
            self._armed = True
            self.get_logger().info(
                f"depth_controller | depth={depth_topic} ping={ping_topic} heave+error={heave_topic} "
                "auto_arm_on_start: ARMED"
            )
        else:
            self.get_logger().info(
                f"depth_controller | depth={depth_topic} ping={ping_topic} heave+error={heave_topic} "
                "(disarmed: heave=0; call arm service)"
            )

    def _on_arm(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data and not self._armed:
            if bool(self.get_parameter("reset_integral_on_arm").value):
                self._integral = 0.0
                self._d_meas_filt = 0.0
        self._armed = bool(request.data)
        response.success = True
        response.message = "armed" if self._armed else "disarmed"
        self.get_logger().info(response.message)
        if not self._armed:
            self._publish_heave_and_error(0.0, 0.0, 0.0)
        return response

    def _on_mission_target_depth(self, msg: Float32) -> None:
        v = float(msg.data)
        if not math.isfinite(v):
            return
        self._mission_target_depth = v
        self._mission_target_stamp = self.get_clock().now()

    def _on_ping(self, msg: Float32) -> None:
        v = float(msg.data)
        if not math.isfinite(v):
            return
        self._last_ping = v
        self._last_ping_time = self.get_clock().now()

    def _ping_guard_correction(self, heave_cmd: float) -> float:
        if not bool(self.get_parameter("use_ping_guard").value):
            return heave_cmd
        if self._last_ping is None:
            return heave_cmd
        dt_ping = (self.get_clock().now() - self._last_ping_time).nanoseconds * 1e-9
        if dt_ping > float(self.get_parameter("ping_stale_sec").value):
            return heave_cmd
        lo = float(self.get_parameter("min_ping_valid_m").value)
        hi = float(self.get_parameter("max_ping_valid_m").value)
        p = self._last_ping
        if p < lo or p > hi:
            return heave_cmd
        floor = float(self.get_parameter("min_clearance_ping_m").value)
        g = float(self.get_parameter("ping_guard_gain").value)
        margin = p - floor
        if margin >= 0.0:
            return heave_cmd
        return heave_cmd + g * margin

    def _on_depth(self, msg: Float32) -> None:
        depth = float(msg.data)
        if not math.isfinite(depth):
            self.get_logger().warn("Non-finite depth; ignoring sample.")
            return

        now = self.get_clock().now()
        dt = (now - self._last_depth_time).nanoseconds * 1e-9
        self._last_depth_time = now
        target = self._resolve_target_depth(now)
        depth_error = target - depth

        if not self._armed:
            self._publish_heave_and_error(0.0, depth_error, depth)
            self._last_depth = depth
            return

        max_dt = float(self.get_parameter("max_depth_dt_sec").value)
        if self._last_depth is None or dt <= 0.0 or dt > max_dt:
            self._last_depth = depth
            self._d_meas_filt = 0.0
            return

        kp = float(self.get_parameter("kp").value)
        ki = float(self.get_parameter("ki").value)
        kd = float(self.get_parameter("kd").value)
        ilim = float(self.get_parameter("integral_limit").value)
        alpha = float(self.get_parameter("depth_derivative_lp_alpha").value)

        e = target - depth
        self._integral += e * dt
        self._integral = clamp(self._integral, -ilim, ilim)

        d_meas_raw = (depth - self._last_depth) / dt
        self._d_meas_filt = alpha * d_meas_raw + (1.0 - alpha) * self._d_meas_filt
        d_term = -kd * self._d_meas_filt

        self._last_depth = depth

        sign = float(self.get_parameter("output_sign").value)
        heave = sign * (kp * e + ki * self._integral + d_term)
        heave = self._ping_guard_correction(heave)
        heave = clamp(
            heave,
            float(self.get_parameter("min_heave_cmd").value),
            float(self.get_parameter("max_heave_cmd").value),
        )
        self._publish_heave_and_error(heave, e, depth)

        if bool(self.get_parameter("enable_debug_logs").value):
            self.get_logger().info(
                f"depth={depth:.3f} tgt={target:.3f} e={e:.3f} heave={heave:.3f} "
                f"i={self._integral:.3f} d_filt={self._d_meas_filt:.3f}"
            )

    def _publish_heave_and_error(self, heave_value: float, depth_error: float, depth: float) -> None:
        m = Vector3()
        m.x = float(self._heave_to_pwm(heave_value))
        m.y = float(depth_error)
        m.z = float(depth)
        self._heave_pub.publish(m)

    def _heave_to_pwm(self, heave_value: float) -> float:
        neutral = float(self.get_parameter("heave_pwm_neutral").value)
        per_unit = float(self.get_parameter("heave_pwm_per_unit").value)
        return neutral + heave_value * per_unit

    def _resolve_target_depth(self, now: Time) -> float:
        target = float(self.get_parameter("target_depth_m").value)
        stale = float(self.get_parameter("mission_target_stale_sec").value)
        if self._mission_target_depth is not None and self._mission_target_stamp is not None:
            if stale <= 0.0:
                target = float(self._mission_target_depth)
            else:
                age = (now - self._mission_target_stamp).nanoseconds * 1e-9
                if age < stale:
                    target = float(self._mission_target_depth)
        return target


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
