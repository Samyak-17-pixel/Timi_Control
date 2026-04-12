"""ROS 2 mother node: odometry in, mission + control, PWM out."""

from __future__ import annotations

import threading
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from timi_auv_control.allocation import allocate_wrench
from timi_auv_control.controllers import WrenchController
from timi_auv_control.geometry import build_allocation_matrix, build_thruster_list, load_geometry
from timi_auv_control.missions import (
    MissionCommand,
    PathFollowingMission,
    StationKeepingMission,
    VehicleState,
    WaypointMission,
)
from timi_auv_control.thruster_model import forces_to_pwm_array


class MotherNode(Node):
    def __init__(self) -> None:
        super().__init__("timi_auv_mother")

        self.declare_parameter("control_config", "")
        self.declare_parameter("geometry_config", "")
        self.declare_parameter("mission_config", "")
        self.declare_parameter("mission_type", "station_keeping")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("pwm_topic", "/auv/thrusters/pwm")
        self.declare_parameter("control_rate_hz", 50.0)

        cpath = self.get_parameter("control_config").get_parameter_value().string_value
        gpath = self.get_parameter("geometry_config").get_parameter_value().string_value
        mpath = self.get_parameter("mission_config").get_parameter_value().string_value
        mtype = self.get_parameter("mission_type").get_parameter_value().string_value

        if not cpath or not gpath or not mpath:
            self.get_logger().fatal(
                "control_config, geometry_config, and mission_config parameters must be set"
            )
            raise RuntimeError("missing config paths")

        with open(self._resolve(cpath), "r", encoding="utf-8") as f:
            self._control_cfg = yaml.safe_load(f)
        geo = load_geometry(self._resolve(gpath))
        self._thrusters = build_thruster_list(geo)
        self._B = build_allocation_matrix(self._thrusters)
        self._n_thr = len(self._thrusters)

        f_lim = float(self._control_cfg.get("thrusters", {}).get("force_limit_n", 55.0))
        self._f_min = np.full(self._n_thr, -f_lim)
        self._f_max = np.full(self._n_thr, f_lim)

        with open(self._resolve(mpath), "r", encoding="utf-8") as f:
            mission_raw = yaml.safe_load(f)

        self._mission = self._make_mission(mtype, mission_raw)
        self._mission.reset()

        self._wc = WrenchController()
        self._wc.configure_from_yaml(self._control_cfg)

        tm = self._control_cfg.get("thrusters", {})
        self._pwm_min = float(tm.get("pwm_min", 1200))
        self._pwm_neutral = float(tm.get("pwm_neutral", 1500))
        self._pwm_max = float(tm.get("pwm_max", 1800))
        self._f_fwd = float(tm.get("forward_max_thrust_n", 51.5))
        self._f_rev = float(tm.get("reverse_max_thrust_n", 40.2))

        self._lock = threading.Lock()
        self._odom: Optional[Odometry] = None
        self._last_cmd: Optional[MissionCommand] = None

        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self._sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        pwm_topic = self.get_parameter("pwm_topic").get_parameter_value().string_value
        self._pub = self.create_publisher(Int32MultiArray, pwm_topic, 10)

        rate = float(self.get_parameter("control_rate_hz").get_parameter_value().double_value)
        period = 1.0 / max(rate, 1.0)
        self._timer = self.create_timer(period, self._on_timer)
        self._dt = period

        self.get_logger().info(
            f"Mother node started: mission={mtype}, rate={rate} Hz, thrusters={self._n_thr}"
        )

    def _resolve(self, p: str) -> Path:
        path = Path(p).expanduser()
        if path.is_file():
            return path
        try:
            from ament_index_python.packages import get_package_share_directory

            share = Path(get_package_share_directory("timi_auv_control"))
            for cand in (share / p, share / "config" / Path(p).name, share / p.name):
                if cand.is_file():
                    return cand
        except Exception:
            pass
        return path

    def _make_mission(self, mtype: str, cfg: dict):
        mtype = mtype.strip().lower()
        if mtype in ("station_keeping", "station", "hover"):
            return StationKeepingMission(cfg)
        if mtype in ("waypoint", "waypoints"):
            return WaypointMission(cfg)
        if mtype in ("path_following", "path", "spline"):
            return PathFollowingMission(cfg)
        raise ValueError(f"Unknown mission_type: {mtype}")

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._odom = msg

    def _on_timer(self) -> None:
        with self._lock:
            msg = self._odom
        if msg is None:
            self._publish_neutral()
            return

        p = msg.pose.pose.position
        p_ned = np.array([p.x, p.y, p.z], dtype=float)
        q = msg.pose.pose.orientation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        tw = msg.twist.twist
        v_body = np.array([tw.linear.x, tw.linear.y, tw.linear.z], dtype=float)
        omega = np.array([tw.angular.x, tw.angular.y, tw.angular.z], dtype=float)

        now = self.get_clock().now().nanoseconds * 1e-9
        state = VehicleState(
            t_wall=now,
            p_ned=p_ned,
            v_body=v_body,
            omega_body=omega,
            q=(qx, qy, qz, qw),
        )

        try:
            cmd = self._mission.step(state, self._dt)
        except Exception as e:
            self.get_logger().error(f"Mission step failed: {e}")
            self._publish_neutral()
            return

        self._last_cmd = cmd
        if cmd.finished:
            self.get_logger().warn("Mission reports finished — publishing neutral PWM")
            self._publish_neutral()
            return

        wrench = self._wc.compute_wrench(
            self._dt,
            p_ned,
            v_body,
            omega,
            (qx, qy, qz, qw),
            cmd.p_des_ned,
            cmd.v_des_ned,
            cmd.roll_des,
            cmd.pitch_des,
            cmd.yaw_des,
            cmd.omega_des_body,
        )

        f, sat = allocate_wrench(wrench, self._B, self._f_min, self._f_max)
        if sat:
            self.get_logger().debug("Thrust allocation saturated")

        pwm = forces_to_pwm_array(
            f,
            self._f_fwd,
            self._f_rev,
            self._pwm_min,
            self._pwm_neutral,
            self._pwm_max,
        )
        out = Int32MultiArray()
        out.data = [int(round(x)) for x in pwm.tolist()]
        self._pub.publish(out)

    def _publish_neutral(self) -> None:
        n = self._n_thr
        pwm = Int32MultiArray()
        pwm.data = [int(self._pwm_neutral)] * n
        self._pub.publish(pwm)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
