import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_srvs.srv import SetBool


class CmdVel3dMerge(Node):
    """
    Merges planar /control/cmd_vel (2D controller) with heave from /control/heave_cmd
    into /control/cmd_vel_3d for thruster_allocator_3d. Roll/pitch rates are zero.

    Optional wall-clock mission timer (from this node's startup): after mission_duration_sec,
    publishes zero cmd_vel_3d until shutdown, and optionally disarms depth_controller once.
    """

    def __init__(self) -> None:
        super().__init__("cmd_vel_3d_merge")

        self.declare_parameter("cmd_vel_2d_topic", "/control/cmd_vel")
        self.declare_parameter("heave_cmd_topic", "/control/heave_cmd")
        self.declare_parameter("cmd_vel_3d_topic", "/control/cmd_vel_3d")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("heave_pwm_neutral", 1500.0)
        self.declare_parameter("heave_pwm_per_unit", 400.0)

        self.declare_parameter("mission_duration_sec", 0.0)
        self.declare_parameter("depth_arm_service", "/depth_controller/arm")
        self.declare_parameter("on_mission_expire_disarm_depth", True)

        cmd2d = str(self.get_parameter("cmd_vel_2d_topic").value)
        heave_t = str(self.get_parameter("heave_cmd_topic").value)
        cmd3d = str(self.get_parameter("cmd_vel_3d_topic").value)
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        period = 1.0 / max(1e-3, rate_hz)

        self._last_2d = Twist()
        self._last_heave = 0.0
        self._mission_start = self.get_clock().now()
        self._mission_expired = False
        self._disarm_sent = False
        arm_svc = str(self.get_parameter("depth_arm_service").value)
        self._arm_client = self.create_client(SetBool, arm_svc)

        self.create_subscription(Twist, cmd2d, self._on_2d, 10)
        self.create_subscription(Vector3, heave_t, self._on_heave, 10)
        self._pub = self.create_publisher(Twist, cmd3d, 10)
        self.create_timer(period, self._on_timer)

        dur = float(self.get_parameter("mission_duration_sec").value)
        if dur > 0.0:
            self.get_logger().info(
                f"cmd_vel_3d_merge | {cmd2d} + {heave_t} -> {cmd3d} @ {rate_hz} Hz | "
                f"mission_timer={dur:.1f}s then neutral cmd + optional disarm"
            )
        else:
            self.get_logger().info(f"cmd_vel_3d_merge | {cmd2d} + {heave_t} -> {cmd3d} @ {rate_hz} Hz")

    def _on_2d(self, msg: Twist) -> None:
        self._last_2d = msg

    def _on_heave(self, msg: Vector3) -> None:
        pwm = float(msg.x)
        neutral = float(self.get_parameter("heave_pwm_neutral").value)
        per_unit = float(self.get_parameter("heave_pwm_per_unit").value)
        self._last_heave = (pwm - neutral) / max(1e-6, per_unit)

    def _on_timer(self) -> None:
        dur = float(self.get_parameter("mission_duration_sec").value)
        if dur > 0.0 and not self._mission_expired:
            elapsed = (self.get_clock().now() - self._mission_start).nanoseconds * 1e-9
            if elapsed >= dur:
                self._mission_expired = True
                self.get_logger().warn(
                    f"Mission duration expired ({dur:.1f}s). Publishing neutral cmd_vel_3d."
                )

        if self._mission_expired:
            out = Twist()
            self._pub.publish(out)
            if (
                bool(self.get_parameter("on_mission_expire_disarm_depth").value)
                and not self._disarm_sent
            ):
                if self._arm_client.service_is_ready():
                    req = SetBool.Request()
                    req.data = False
                    self._arm_client.call_async(req)
                    self._disarm_sent = True
                    self.get_logger().info("Depth controller disarm requested on mission expiry.")
            return

        out = Twist()
        out.linear.x = float(self._last_2d.linear.x)
        out.linear.y = float(self._last_2d.linear.y)
        out.linear.z = float(self._last_heave)
        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = float(self._last_2d.angular.z)
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVel3dMerge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
