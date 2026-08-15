"""
Qualification gate mission: depth hold → approach → align → pass → clear distance →
180° → return → second align/pass → forward ≥ post_second_pass_forward_m → surface → idle.

Publishes geometry_msgs/Twist on /control/cmd_vel (same convention as auv_2d_control).
Depth setpoint on mission_target_depth_topic; optional arm via service if auto_arm is off.
"""

from __future__ import annotations

import math
from enum import Enum, auto
from typing import Any, Optional

import rclpy
from rclpy.time import Time
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool

from dvl_msgs.msg import DVLDR
from sbg_driver.msg import SbgImuData

from qualification_gate_interfaces.msg import GateDetection


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class MissionState(Enum):
    DEPTH_DIVE = auto()
    APPROACH = auto()
    ALIGN = auto()
    PASS = auto()
    CLEAR_DISTANCE = auto()
    TURN_AROUND = auto()
    RETURN_APPROACH = auto()
    SECOND_ALIGN = auto()
    SECOND_PASS = auto()
    SECOND_CLEAR_DISTANCE = auto()
    SURFACE = auto()
    IDLE = auto()


class GateMissionNode(Node):
    def __init__(self) -> None:
        super().__init__("gate_mission")

        # --- Topics (match auv_2d_control conventions) ---
        self.declare_parameter("cmd_vel_topic", "/control/cmd_vel")
        self.declare_parameter("gate_topic", "/gate/detection")
        self.declare_parameter("dvl_topic", "/dvl/position")
        self.declare_parameter("status_topic", "/qualification/mission/status")

        # --- Depth (monitoring optional; hold is external) ---
        self.declare_parameter("depth_topic", "/auv/depth")
        self.declare_parameter("mission_target_depth_m", 0.5)
        self.declare_parameter("mission_target_depth_topic", "/qualification/mission/target_depth")
        self.declare_parameter("publish_mission_target_depth", True)
        self.declare_parameter("skip_depth_acquisition", False)
        self.declare_parameter("depth_acquire_tolerance_m", 0.1)
        # Hold at target depth this long (s) before APPROACH; if 0, use depth_acquire_hold_frames instead.
        self.declare_parameter("depth_acquire_hold_sec", 5.0)
        self.declare_parameter("depth_acquire_hold_frames", 20)
        self.declare_parameter("depth_acquire_timeout_sec", 120.0)

        # --- SBG IMU (gyro damping for turn / optional vision yaw) ---
        self.declare_parameter("sbg_imu_topic", "/sbg/imu_data")
        self.declare_parameter("use_sbg_imu_turn_damping", True)
        self.declare_parameter("turn_yaw_kd_imu", 0.35)
        self.declare_parameter("turn_imu_gyro_z_sign", 1.0)
        self.declare_parameter("use_sbg_imu_vision_yaw_damping", True)
        self.declare_parameter("align_yaw_kd_imu", 0.12)
        self.declare_parameter("align_imu_gyro_z_sign", 1.0)

        # --- Mission geometry / speeds ---
        self.declare_parameter("surge_approach_m_s", 0.5)
        self.declare_parameter("surge_align_m_s", 0.25)
        self.declare_parameter("surge_pass_m_s", 0.5)
        self.declare_parameter("surge_return_m_s", 0.5)
        self.declare_parameter("post_gate_forward_m", 3.0)
        # After return pass: drive at least this far (m) before surfacing.
        self.declare_parameter("post_second_pass_forward_m", 5.0)
        # Mission publishes this depth (m) on mission_target_depth_topic during SURFACE before IDLE.
        self.declare_parameter("surface_depth_m", 0.12)
        self.declare_parameter("surface_hold_sec", 8.0)
        self.declare_parameter("dvl_yaw_in_degrees", False)

        # --- Horizontal distance from DVL position (x,y in DVL frame) ---
        self.declare_parameter("dvl_use_horizontal_distance", True)

        # --- Vision alignment (camera vs gate centre); use float defaults (YAML may use 18.0) ---
        self.declare_parameter("camera_image_width_px", 640.0)
        self.declare_parameter("align_deadband_px", 18.0)
        self.declare_parameter("align_fine_deadband_px", 8.0)
        self.declare_parameter("yaw_only_if_error_above_px", 22.0)
        self.declare_parameter("kp_sway_per_px", 0.0012)
        self.declare_parameter("kp_yaw_per_px", 0.003)
        self.declare_parameter("max_sway_m_s", 0.28)
        self.declare_parameter("max_yaw_rate_rad_s", 0.35)
        self.declare_parameter("aligned_consecutive_frames", 4)
        self.declare_parameter("one_pole_scan_angle_deg", 30.0)
        self.declare_parameter("one_pole_scan_yaw_rate_rad_s", 0.30)
        self.declare_parameter("one_pole_scan_yaw_kp", 1.4)
        self.declare_parameter("one_pole_scan_done_err_rad", 0.10)

        # --- Turn in place ---
        self.declare_parameter("turn_yaw_kp", 1.8)
        self.declare_parameter("turn_yaw_max_rate_rad_s", 0.45)
        self.declare_parameter("turn_done_err_rad", 0.12)

        # --- Timers / safety ---
        self.declare_parameter("approach_timeout_sec", 120.0)
        self.declare_parameter("align_timeout_sec", 90.0)
        self.declare_parameter("return_search_timeout_sec", 120.0)
        self.declare_parameter("mission_start_delay_sec", 0.0)
        self.declare_parameter("pass_timeout_sec", 0.0)
        # 0 disables global mission timeout; if > 0, force SURFACE after this many seconds.
        self.declare_parameter("mission_timeout_sec", 0.0)

        # --- Depth arm via service (usually unnecessary if depth_controller auto_arm_on_start is true) ---
        self.declare_parameter("arm_depth_on_start", False)
        self.declare_parameter("depth_arm_service", "/depth_controller/arm")

        # --- Rates ---
        self.declare_parameter("control_rate_hz", 20.0)

        self._state = (
            MissionState.APPROACH
            if bool(self.get_parameter("skip_depth_acquisition").value)
            else MissionState.DEPTH_DIVE
        )
        self._mission_started = False
        self._start_time = self.get_clock().now()
        self._depth_good_streak = 0
        self._depth_in_tol_since: Optional[Time] = None
        self._depth_phase_t0: Optional[Time] = None

        self._gate: Optional[GateDetection] = None
        self._depth: Optional[float] = None
        self._dvl: Optional[DVLDR] = None
        self._sbg_imu: Optional[SbgImuData] = None

        self._had_gate_center_valid = False
        self._align_good_streak = 0

        self._clear_ref_xy: Optional[tuple[float, float]] = None
        self._second_clear_ref_xy: Optional[tuple[float, float]] = None
        self._yaw_at_turn_start: Optional[float] = None
        self._surface_enter_time: Optional[Time] = None
        self._idle_after_surface = False

        self._second_leg = False
        self._align_enter_time: Optional[Time] = None
        self._pass_enter_time: Optional[Time] = None
        self._return_start_time: Optional[Time] = None
        self._approach_start_time: Optional[Time] = None
        self._start_delay_timer = None
        self._align_scan_phase = "left"
        self._align_scan_ref_yaw: Optional[float] = None

        # Only create a client if we will call the arm service (default false when depth_controller
        # uses auto_arm_on_start). No client => no service discovery and no stale-code arm spam.
        self._depth_arm_client: Any = None
        self._depth_arm_request_sent = False
        self._depth_arm_waiting_logged = False
        if bool(self.get_parameter("arm_depth_on_start").value):
            self._depth_arm_client = self.create_client(
                SetBool, str(self.get_parameter("depth_arm_service").value)
            )
            self.get_logger().info(
                "arm_depth_on_start: will call %s when service is ready"
                % str(self.get_parameter("depth_arm_service").value)
            )
        else:
            self.get_logger().info(
                "arm_depth_on_start: false — depth_controller should arm via auto_arm_on_start "
                "(no SetBool from gate_mission)."
            )

        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._pub_cmd = self.create_publisher(Twist, cmd_topic, 10)
        self._pub_status = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)
        self._pub_target_depth = self.create_publisher(
            Float32,
            str(self.get_parameter("mission_target_depth_topic").value),
            10,
        )

        self.create_subscription(
            GateDetection,
            str(self.get_parameter("gate_topic").value),
            self._on_gate,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("depth_topic").value),
            self._on_depth,
            10,
        )
        self.create_subscription(
            DVLDR,
            str(self.get_parameter("dvl_topic").value),
            self._on_dvl,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            SbgImuData,
            str(self.get_parameter("sbg_imu_topic").value),
            self._on_sbg_imu,
            qos_profile_sensor_data,
        )

        rate = float(self.get_parameter("control_rate_hz").value)
        period = 1.0 / max(1.0, rate)
        self._timer = self.create_timer(period, self._tick)

        delay = float(self.get_parameter("mission_start_delay_sec").value)
        if delay > 0.0:
            self._start_delay_timer = self.create_timer(delay, self._on_start_delay)
        else:
            self._begin_mission()

        self.get_logger().info(
            "gate_mission | cmd=%s gate=%s dvl=%s imu=%s"
            % (
                cmd_topic,
                str(self.get_parameter("gate_topic").value),
                str(self.get_parameter("dvl_topic").value),
                str(self.get_parameter("sbg_imu_topic").value),
            )
        )

    def _on_start_delay(self) -> None:
        if self._start_delay_timer is not None:
            self._start_delay_timer.cancel()
            self._start_delay_timer = None
        self._begin_mission()

    def _begin_mission(self) -> None:
        if self._mission_started:
            return
        self._mission_started = True
        self._start_time = self.get_clock().now()

    def _try_arm_depth_controller(self) -> None:
        """Non-blocking: retry each tick until the arm service is discovered and the call is sent."""
        if self._depth_arm_client is None:
            return
        if not bool(self.get_parameter("arm_depth_on_start").value):
            return
        if self._depth_arm_request_sent:
            return
        cli = self._depth_arm_client
        if not cli.service_is_ready():
            if not self._depth_arm_waiting_logged:
                self.get_logger().info(
                    "Waiting for depth arm service (will retry): %s"
                    % str(self.get_parameter("depth_arm_service").value)
                )
                self._depth_arm_waiting_logged = True
            return
        self._depth_arm_request_sent = True
        fut = cli.call_async(SetBool.Request(data=True))

        def _done(_f) -> None:
            self.get_logger().info("Depth controller armed.")

        fut.add_done_callback(_done)

    def _publish_mission_target_depth(self) -> None:
        if not bool(self.get_parameter("publish_mission_target_depth").value):
            return
        m = Float32()
        if self._state == MissionState.SURFACE:
            m.data = float(self.get_parameter("surface_depth_m").value)
        elif self._state == MissionState.IDLE and self._idle_after_surface:
            # Keep publishing shallow setpoint so depth PID / heave stay active after mission end.
            m.data = float(self.get_parameter("surface_depth_m").value)
        else:
            m.data = float(self.get_parameter("mission_target_depth_m").value)
        self._pub_target_depth.publish(m)

    def _on_gate(self, msg: GateDetection) -> None:
        self._gate = msg

    def _on_depth(self, msg: Float32) -> None:
        self._depth = float(msg.data)

    def _on_dvl(self, msg: DVLDR) -> None:
        self._dvl = msg

    def _on_sbg_imu(self, msg: SbgImuData) -> None:
        self._sbg_imu = msg

    def _dampen_yaw_with_gyro(self, yaw_cmd: float) -> float:
        """Subtract measured body yaw rate (gyro z) from vision yaw command for damping."""
        if not bool(self.get_parameter("use_sbg_imu_vision_yaw_damping").value):
            return yaw_cmd
        if self._sbg_imu is None:
            return yaw_cmd
        kd = float(self.get_parameter("align_yaw_kd_imu").value)
        sgn = float(self.get_parameter("align_imu_gyro_z_sign").value)
        gz = float(self._sbg_imu.gyro.z)
        out = yaw_cmd - kd * sgn * gz
        mx = float(self.get_parameter("max_yaw_rate_rad_s").value)
        return clamp(out, -mx, mx)

    def _dvl_yaw_rad(self, msg: DVLDR) -> float:
        y = float(msg.yaw)
        if bool(self.get_parameter("dvl_yaw_in_degrees").value):
            y = math.radians(y)
        return y

    def _horizontal_distance_from_start(self, msg: DVLDR, ref: tuple[float, float]) -> float:
        px = float(msg.position.x)
        py = float(msg.position.y)
        if bool(self.get_parameter("dvl_use_horizontal_distance").value):
            return math.hypot(px - ref[0], py - ref[1])
        return abs(px - ref[0])

    def _vision_sway_yaw(self, gate: GateDetection) -> tuple[float, float]:
        """Returns (sway_cmd, yaw_rate_cmd) from center_error_px."""
        if not gate.gate_center_valid:
            return 0.0, 0.0
        err = float(gate.center_error_px)
        dead = float(self.get_parameter("align_deadband_px").value)
        fine = float(self.get_parameter("align_fine_deadband_px").value)
        if abs(err) <= fine:
            return 0.0, 0.0
        if abs(err) <= dead:
            err = 0.0

        kp_s = float(self.get_parameter("kp_sway_per_px").value)
        kp_y = float(self.get_parameter("kp_yaw_per_px").value)
        max_v = float(self.get_parameter("max_sway_m_s").value)
        max_r = float(self.get_parameter("max_yaw_rate_rad_s").value)
        yaw_thr = float(self.get_parameter("yaw_only_if_error_above_px").value)

        sway = -kp_s * err
        sway = clamp(sway, -max_v, max_v)

        yaw_r = 0.0
        if abs(err) >= yaw_thr:
            yaw_r = -kp_y * err
            yaw_r = clamp(yaw_r, -max_r, max_r)

        return sway, yaw_r

    def _publish_cmd(self, surge: float, sway: float, yaw_rate: float) -> None:
        t = Twist()
        t.linear.x = float(surge)
        t.linear.y = float(sway)
        t.angular.z = float(yaw_rate)
        self._pub_cmd.publish(t)

    def _publish_status(self, text: str) -> None:
        m = String()
        m.data = text
        self._pub_status.publish(m)

    def _has_both_poles(self, gate: Optional[GateDetection]) -> bool:
        return bool(gate is not None and gate.pole1_detected and gate.pole2_detected)

    def _has_one_pole(self, gate: Optional[GateDetection]) -> bool:
        return bool(gate is not None and (gate.pole1_detected ^ gate.pole2_detected))

    def _reset_one_pole_scan(self) -> None:
        self._align_scan_phase = "left"
        self._align_scan_ref_yaw = None

    def _one_pole_scan_yaw_cmd(self, dvl: Optional[DVLDR]) -> float:
        max_rate = float(self.get_parameter("one_pole_scan_yaw_rate_rad_s").value)
        if dvl is None:
            return max_rate if self._align_scan_phase == "left" else -max_rate

        yaw_now = self._dvl_yaw_rad(dvl)
        if self._align_scan_ref_yaw is None:
            self._align_scan_ref_yaw = yaw_now

        angle = math.radians(float(self.get_parameter("one_pole_scan_angle_deg").value))
        kp = float(self.get_parameter("one_pole_scan_yaw_kp").value)
        done = float(self.get_parameter("one_pole_scan_done_err_rad").value)
        sign = 1.0 if self._align_scan_phase == "left" else -1.0
        target = wrap_to_pi(float(self._align_scan_ref_yaw) + sign * angle)
        err = wrap_to_pi(target - yaw_now)
        yaw_cmd = clamp(kp * err, -max_rate, max_rate)

        if abs(err) <= done:
            if self._align_scan_phase == "left":
                self._align_scan_phase = "right"
                self._align_scan_ref_yaw = yaw_now
                self.get_logger().info("One-pole scan: left sweep complete, scanning right.")
            else:
                self._align_scan_phase = "left"
                self._align_scan_ref_yaw = yaw_now
                self.get_logger().info("One-pole scan: right sweep complete, scanning left.")
            return 0.0

        return yaw_cmd

    def _transition(self, new: MissionState, reason: str) -> None:
        prev = self._state
        self.get_logger().info("state %s -> %s (%s)" % (self._state.name, new.name, reason))
        self._state = new
        self._publish_status("%s: %s" % (new.name, reason))
        tnow = self.get_clock().now()
        if new == MissionState.APPROACH:
            self._approach_start_time = tnow
        if new in (MissionState.ALIGN, MissionState.SECOND_ALIGN):
            self._align_enter_time = tnow
            self._reset_one_pole_scan()
        if new == MissionState.PASS:
            self._pass_enter_time = tnow
        if new == MissionState.RETURN_APPROACH:
            self._return_start_time = tnow
        if new == MissionState.IDLE:
            self._idle_after_surface = prev == MissionState.SURFACE

    def _tick(self) -> None:
        if not self._mission_started:
            return

        self._try_arm_depth_controller()

        now = self.get_clock().now()
        mission_timeout = float(self.get_parameter("mission_timeout_sec").value)
        if mission_timeout > 0.0:
            elapsed = (now - self._start_time).nanoseconds * 1e-9
            if elapsed >= mission_timeout and self._state not in (MissionState.SURFACE, MissionState.IDLE):
                self._surface_enter_time = None
                self._transition(MissionState.SURFACE, "global mission timeout")
                self._publish_cmd(0.0, 0.0, 0.0)
                return

        g = self._gate
        dvl = self._dvl

        st = self._state
        if bool(self.get_parameter("publish_mission_target_depth").value):
            # Publish every tick for entire mission (including IDLE) so depth_controller never
            # loses the mission setpoint; working depth until SURFACE, then shallow setpoint.
            self._publish_mission_target_depth()
        # Heartbeat status so operators always see live mission state on topic.
        self._publish_status(st.name)
        # Heartbeat status so operators always see live mission state on topic.
        self._publish_status(st.name)

        # Defaults
        surge, sway, yaw_r = 0.0, 0.0, 0.0

        if st == MissionState.DEPTH_DIVE:
            if self._depth_phase_t0 is None:
                self._depth_phase_t0 = now
            surge, sway, yaw_r = 0.0, 0.0, 0.0
            tgt = float(self.get_parameter("mission_target_depth_m").value)
            tol = float(self.get_parameter("depth_acquire_tolerance_m").value)
            hold_sec = float(self.get_parameter("depth_acquire_hold_sec").value)
            if self._depth is not None and math.isfinite(self._depth):
                if abs(float(self._depth) - tgt) <= tol:
                    if self._depth_in_tol_since is None:
                        self._depth_in_tol_since = now
                    if hold_sec > 0.0:
                        if (now - self._depth_in_tol_since).nanoseconds * 1e-9 >= hold_sec:
                            self._transition(MissionState.APPROACH, "depth held for depth_acquire_hold_sec")
                            self._publish_cmd(0.0, 0.0, 0.0)
                            return
                    else:
                        self._depth_good_streak += 1
                        need = int(self.get_parameter("depth_acquire_hold_frames").value)
                        if self._depth_good_streak >= need:
                            self._transition(MissionState.APPROACH, "depth within tolerance (frames)")
                            self._publish_cmd(0.0, 0.0, 0.0)
                            return
                else:
                    self._depth_in_tol_since = None
                    self._depth_good_streak = 0
            dtmo = float(self.get_parameter("depth_acquire_timeout_sec").value)
            if self._depth_phase_t0 is not None and (now - self._depth_phase_t0).nanoseconds * 1e-9 > dtmo:
                self.get_logger().warn("DEPTH_DIVE timeout; continuing to APPROACH")
                self._transition(MissionState.APPROACH, "depth acquire timeout")
            self._publish_cmd(surge, sway, yaw_r)
            return

        if st == MissionState.APPROACH:
            surge = float(self.get_parameter("surge_approach_m_s").value)
            t_approach = self._approach_start_time if self._approach_start_time is not None else now
            if self._has_both_poles(g):
                self._transition(MissionState.ALIGN, "both poles visible")
            elif self._has_one_pole(g):
                self._transition(MissionState.ALIGN, "single pole detected; scanning for second")
                self._publish_cmd(0.0, 0.0, 0.0)
                return
            elif (now - t_approach).nanoseconds * 1e-9 > float(self.get_parameter("approach_timeout_sec").value):
                self._transition(MissionState.IDLE, "approach timeout")
            self._publish_cmd(surge, sway, yaw_r)
            return

        if st == MissionState.ALIGN:
            t_align_start = self._align_enter_time
            if t_align_start is None:
                self._align_enter_time = now
                t_align_start = now
            if g is None:
                self._publish_cmd(float(self.get_parameter("surge_align_m_s").value), 0.0, 0.0)
                return
            if self._has_one_pole(g):
                yaw_r = self._one_pole_scan_yaw_cmd(dvl)
                self._publish_cmd(0.0, 0.0, yaw_r)
                return
            if self._has_both_poles(g) and g.gate_center_valid:
                self._had_gate_center_valid = True
                sway, yaw_r = self._vision_sway_yaw(g)
                err = abs(float(g.center_error_px))
                thr = float(self.get_parameter("align_fine_deadband_px").value)
                if err <= thr:
                    self._align_good_streak += 1
                else:
                    self._align_good_streak = 0
                need = int(self.get_parameter("aligned_consecutive_frames").value)
                if self._align_good_streak >= need:
                    self._transition(MissionState.PASS, "aligned (vision)")
            else:
                self._align_good_streak = 0
                sway, yaw_r = 0.0, 0.0
            surge = float(self.get_parameter("surge_align_m_s").value)
            if (now - t_align_start).nanoseconds * 1e-9 > float(self.get_parameter("align_timeout_sec").value):
                self._transition(MissionState.PASS, "align timeout, continue")
            yaw_r = self._dampen_yaw_with_gyro(yaw_r)
            self._publish_cmd(surge, sway, yaw_r)
            return

        if st == MissionState.PASS:
            surge = float(self.get_parameter("surge_pass_m_s").value)
            pto = float(self.get_parameter("pass_timeout_sec").value)
            if pto > 0.0 and self._pass_enter_time is not None:
                if (now - self._pass_enter_time).nanoseconds * 1e-9 > pto:
                    self._transition(MissionState.IDLE, "pass timeout")
                    self._publish_cmd(0.0, 0.0, 0.0)
                    return
            if g is not None:
                if g.gate_center_valid:
                    self._had_gate_center_valid = True
                    sway, yaw_r = self._vision_sway_yaw(g)
                elif self._had_gate_center_valid:
                    # Gate centre lost after we had a valid centre → start post-gate run
                    self._transition(MissionState.CLEAR_DISTANCE, "gate_center_valid false after true")
                    if dvl is not None:
                        self._clear_ref_xy = (
                            float(dvl.position.x),
                            float(dvl.position.y),
                        )
                    else:
                        self.get_logger().warn("No DVL at CLEAR_DISTANCE start; distance may be wrong.")
                        self._clear_ref_xy = (0.0, 0.0)
                    self._had_gate_center_valid = False
            yaw_r = self._dampen_yaw_with_gyro(yaw_r)
            self._publish_cmd(surge, sway, yaw_r)
            return

        if st == MissionState.CLEAR_DISTANCE:
            surge = float(self.get_parameter("surge_pass_m_s").value)
            if self._clear_ref_xy is None and dvl is not None:
                self._clear_ref_xy = (float(dvl.position.x), float(dvl.position.y))
            target = float(self.get_parameter("post_gate_forward_m").value)
            if dvl is not None and self._clear_ref_xy is not None:
                dist = self._horizontal_distance_from_start(dvl, self._clear_ref_xy)
                if dist >= target:
                    self._clear_ref_xy = None
                    if dvl is not None:
                        self._yaw_at_turn_start = self._dvl_yaw_rad(dvl)
                    self._transition(MissionState.TURN_AROUND, "post-gate distance reached")
            self._publish_cmd(surge, 0.0, 0.0)
            return

        if st == MissionState.TURN_AROUND:
            if dvl is None:
                self._publish_cmd(0.0, 0.0, 0.0)
                return
            y0 = self._yaw_at_turn_start
            if y0 is None:
                y0 = self._dvl_yaw_rad(dvl)
                self._yaw_at_turn_start = y0
            y = self._dvl_yaw_rad(dvl)
            target = wrap_to_pi(y0 + math.pi)
            err = wrap_to_pi(target - y)
            kp = float(self.get_parameter("turn_yaw_kp").value)
            mx = float(self.get_parameter("turn_yaw_max_rate_rad_s").value)
            tol = float(self.get_parameter("turn_done_err_rad").value)
            yaw_r = clamp(kp * err, -mx, mx)
            if bool(self.get_parameter("use_sbg_imu_turn_damping").value) and self._sbg_imu is not None:
                kd = float(self.get_parameter("turn_yaw_kd_imu").value)
                sgn = float(self.get_parameter("turn_imu_gyro_z_sign").value)
                gz = float(self._sbg_imu.gyro.z)
                yaw_r = clamp(yaw_r - kd * sgn * gz, -mx, mx)
            if abs(err) < tol:
                self._publish_cmd(0.0, 0.0, 0.0)
                self._yaw_at_turn_start = None
                self._second_leg = True
                self._align_good_streak = 0
                self._had_gate_center_valid = False
                self._transition(MissionState.RETURN_APPROACH, "turn complete")
                return
            self._publish_cmd(0.0, 0.0, yaw_r)
            return

        if st == MissionState.RETURN_APPROACH:
            surge = float(self.get_parameter("surge_return_m_s").value)
            t0 = self._return_start_time
            if t0 is None:
                self._return_start_time = now
                t0 = now
            if self._has_both_poles(g):
                self._return_start_time = None
                self._transition(MissionState.SECOND_ALIGN, "gate visible on return")
            elif self._has_one_pole(g):
                self._return_start_time = None
                self._transition(MissionState.SECOND_ALIGN, "single pole detected on return; scanning")
                self._publish_cmd(0.0, 0.0, 0.0)
                return
            elif (now - t0).nanoseconds * 1e-9 > float(self.get_parameter("return_search_timeout_sec").value):
                self._return_start_time = None
                self._transition(MissionState.IDLE, "return search timeout")
            self._publish_cmd(surge, 0.0, 0.0)
            return

        if st == MissionState.SECOND_ALIGN:
            if g is None:
                self._publish_cmd(float(self.get_parameter("surge_align_m_s").value), 0.0, 0.0)
                return
            if self._has_one_pole(g):
                yaw_r = self._one_pole_scan_yaw_cmd(dvl)
                self._publish_cmd(0.0, 0.0, yaw_r)
                return
            if self._has_both_poles(g) and g.gate_center_valid:
                self._had_gate_center_valid = True
                sway, yaw_r = self._vision_sway_yaw(g)
                err = abs(float(g.center_error_px))
                thr = float(self.get_parameter("align_fine_deadband_px").value)
                if err <= thr:
                    self._align_good_streak += 1
                else:
                    self._align_good_streak = 0
                need = int(self.get_parameter("aligned_consecutive_frames").value)
                if self._align_good_streak >= need:
                    self._transition(MissionState.SECOND_PASS, "aligned second leg")
            else:
                sway, yaw_r = 0.0, 0.0
            surge = float(self.get_parameter("surge_align_m_s").value)
            yaw_r = self._dampen_yaw_with_gyro(yaw_r)
            self._publish_cmd(surge, sway, yaw_r)
            return

        if st == MissionState.SECOND_PASS:
            surge = float(self.get_parameter("surge_pass_m_s").value)
            if g is not None:
                if g.gate_center_valid:
                    self._had_gate_center_valid = True
                    sway, yaw_r = self._vision_sway_yaw(g)
                elif self._had_gate_center_valid:
                    self._transition(MissionState.SECOND_CLEAR_DISTANCE, "second pass: gate_center lost")
                    if dvl is not None:
                        self._second_clear_ref_xy = (
                            float(dvl.position.x),
                            float(dvl.position.y),
                        )
                    else:
                        self.get_logger().warn("No DVL at SECOND_CLEAR_DISTANCE; distance may be wrong.")
                        self._second_clear_ref_xy = (0.0, 0.0)
                    self._had_gate_center_valid = False
                    self._publish_cmd(surge, sway, yaw_r)
                    return
            yaw_r = self._dampen_yaw_with_gyro(yaw_r)
            self._publish_cmd(surge, sway, yaw_r)
            return

        if st == MissionState.SECOND_CLEAR_DISTANCE:
            surge = float(self.get_parameter("surge_pass_m_s").value)
            if self._second_clear_ref_xy is None and dvl is not None:
                self._second_clear_ref_xy = (float(dvl.position.x), float(dvl.position.y))
            target_d = float(self.get_parameter("post_second_pass_forward_m").value)
            if dvl is not None and self._second_clear_ref_xy is not None:
                dist = self._horizontal_distance_from_start(dvl, self._second_clear_ref_xy)
                if dist >= target_d:
                    self._second_clear_ref_xy = None
                    self._surface_enter_time = None
                    self._transition(MissionState.SURFACE, "post-return distance reached; surfacing")
            self._publish_cmd(surge, 0.0, 0.0)
            return

        if st == MissionState.SURFACE:
            if self._surface_enter_time is None:
                self._surface_enter_time = now
            sh = float(self.get_parameter("surface_hold_sec").value)
            if (now - self._surface_enter_time).nanoseconds * 1e-9 >= sh:
                self._transition(MissionState.IDLE, "surface hold complete")
                self._publish_cmd(0.0, 0.0, 0.0)
                return
            self._publish_cmd(0.0, 0.0, 0.0)
            return

        # IDLE
        self._publish_cmd(0.0, 0.0, 0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GateMissionNode()
    try:
        from rclpy.executors import MultiThreadedExecutor

        exe = MultiThreadedExecutor(num_threads=4)
        exe.add_node(node)
        exe.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
