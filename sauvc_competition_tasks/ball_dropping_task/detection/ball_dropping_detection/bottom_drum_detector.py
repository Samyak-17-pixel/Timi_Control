from __future__ import annotations

import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .common import clamp, status_json


class BottomDrumDetector(Node):
    def __init__(self) -> None:
        super().__init__("bottom_drum_detector")

        self.declare_parameter("image_topic", "/bottom/image_raw")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("image_width", 640.0)
        self.declare_parameter("image_height", 480.0)

        self.declare_parameter("blue_h_low", 85)
        self.declare_parameter("blue_h_high", 145)
        self.declare_parameter("blue_s_low", 45)
        self.declare_parameter("blue_v_low", 35)
        self.declare_parameter("red1_h_low", 0)
        self.declare_parameter("red1_h_high", 15)
        self.declare_parameter("red2_h_low", 160)
        self.declare_parameter("red2_h_high", 179)
        self.declare_parameter("red_s_low", 45)
        self.declare_parameter("red_v_low", 35)
        self.declare_parameter("open_kernel", 5)
        self.declare_parameter("close_kernel", 9)
        self.declare_parameter("min_area_px", 450.0)
        self.declare_parameter("min_circularity", 0.25)
        self.declare_parameter("align_tol_px", 14.0)
        self.declare_parameter("stable_frames_required", 3)
        self.declare_parameter("ema_alpha", 0.45)

        self._bridge = CvBridge()
        self._last_center = Vector3Stamped()
        self._last_error = Vector3Stamped()
        self._last_status = String()
        self._last_debug: Optional[np.ndarray] = None
        self._last_image_time = 0.0
        self._fx: Optional[float] = None
        self._fy: Optional[float] = None
        self._stable_count = 0

        self._pub_center = self.create_publisher(Vector3Stamped, "/ball_dropping/bottom/center", 10)
        self._pub_error = self.create_publisher(Vector3Stamped, "/ball_dropping/bottom/blue_target", 10)
        self._pub_red = self.create_publisher(String, "/ball_dropping/bottom/red_candidates", 10)
        self._pub_status = self.create_publisher(String, "/ball_dropping/bottom/status", 10)
        self._pub_debug = self.create_publisher(Image, "/ball_dropping/bottom/debug_image", 10)
        self._last_red = String()

        image_topic = str(self.get_parameter("image_topic").value)
        self.create_subscription(Image, image_topic, self._on_image, 10)
        hz = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(1.0, hz), self._on_publish_timer)
        self.get_logger().info(f"bottom_drum_detector | image={image_topic}")

    def _on_image(self, msg: Image) -> None:
        self._last_image_time = time.time()
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self._last_status.data = status_json(stage="bottom", ok=False, reason=f"cv_bridge_error:{exc}")
            return

        center_msg, error_msg, status_msg, debug = self._detect(frame)
        self._last_center = center_msg
        self._last_error = error_msg
        self._last_status = status_msg
        self._last_debug = debug

    def _on_publish_timer(self) -> None:
        self._pub_center.publish(self._last_center)
        self._pub_error.publish(self._last_error)
        self._pub_red.publish(self._last_red)
        self._pub_status.publish(self._last_status)
        if bool(self.get_parameter("publish_debug_image").value) and self._last_debug is not None:
            self._pub_debug.publish(self._bridge.cv2_to_imgmsg(self._last_debug, encoding="bgr8"))

        dt = time.time() - self._last_image_time
        if self._last_image_time > 0.0 and dt > 2.0:
            self._last_status.data = status_json(stage="bottom", ok=False, reason="no_image", age_sec=round(dt, 2))

    def _detect(self, frame_bgr: np.ndarray) -> tuple[Vector3Stamped, Vector3Stamped, String, np.ndarray]:
        h, w = frame_bgr.shape[:2]
        cx_ref = 0.5 * w
        cy_ref = 0.5 * h
        dbg = frame_bgr.copy()

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        lo = (
            int(self.get_parameter("blue_h_low").value),
            int(self.get_parameter("blue_s_low").value),
            int(self.get_parameter("blue_v_low").value),
        )
        hi = (
            int(self.get_parameter("blue_h_high").value),
            255,
            255,
        )
        red1_lo = (
            int(self.get_parameter("red1_h_low").value),
            int(self.get_parameter("red_s_low").value),
            int(self.get_parameter("red_v_low").value),
        )
        red1_hi = (
            int(self.get_parameter("red1_h_high").value),
            255,
            255,
        )
        red2_lo = (
            int(self.get_parameter("red2_h_low").value),
            int(self.get_parameter("red_s_low").value),
            int(self.get_parameter("red_v_low").value),
        )
        red2_hi = (
            int(self.get_parameter("red2_h_high").value),
            255,
            255,
        )
        mask = cv2.inRange(hsv, lo, hi)
        red_mask = cv2.bitwise_or(cv2.inRange(hsv, red1_lo, red1_hi), cv2.inRange(hsv, red2_lo, red2_hi))

        k_open = int(self.get_parameter("open_kernel").value)
        k_close = int(self.get_parameter("close_kernel").value)
        k_open = max(1, k_open + (1 - k_open % 2))
        k_close = max(1, k_close + (1 - k_close % 2))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((k_open, k_open), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((k_close, k_close), np.uint8))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, np.ones((k_open, k_open), np.uint8))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, np.ones((k_close, k_close), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_score = -1e9
        min_area = float(self.get_parameter("min_area_px").value)
        min_circ = float(self.get_parameter("min_circularity").value)

        for c in contours:
            area = float(cv2.contourArea(c))
            if area < min_area:
                continue
            per = float(cv2.arcLength(c, True))
            if per <= 1e-6:
                continue
            circ = float(4.0 * np.pi * area / (per * per))
            if circ < min_circ:
                continue
            m = cv2.moments(c)
            if abs(m["m00"]) < 1e-6:
                continue
            cx = float(m["m10"] / m["m00"])
            cy = float(m["m01"] / m["m00"])
            score = area + 1000.0 * circ
            if score > best_score:
                best_score = score
                best = (c, cx, cy, area, circ)

        center_msg = Vector3Stamped()
        center_msg.header.stamp = self.get_clock().now().to_msg()
        center_msg.vector.x = -1.0
        center_msg.vector.y = -1.0
        center_msg.vector.z = 0.0

        error_msg = Vector3Stamped()
        error_msg.header.stamp = center_msg.header.stamp
        error_msg.vector.x = 0.0
        error_msg.vector.y = 0.0
        error_msg.vector.z = 0.0

        aligned = False
        if best is not None:
            c, cx, cy, area, circ = best
            if self._fx is None:
                self._fx = cx
                self._fy = cy
            else:
                a = float(self.get_parameter("ema_alpha").value)
                self._fx = a * cx + (1.0 - a) * self._fx
                self._fy = a * cy + (1.0 - a) * self._fy

            ex = float(self._fx - cx_ref)
            ey = float(self._fy - cy_ref)
            err_px = float(np.hypot(ex, ey))
            conf = clamp((area / float(w * h)) * 5.0 + circ * 0.5, 0.0, 1.0)

            center_msg.vector.x = float(self._fx)
            center_msg.vector.y = float(self._fy)
            center_msg.vector.z = conf
            error_msg.vector.x = ex
            error_msg.vector.y = ey
            error_msg.vector.z = err_px

            tol = float(self.get_parameter("align_tol_px").value)
            if err_px <= tol:
                self._stable_count += 1
            else:
                self._stable_count = 0
            aligned = self._stable_count >= int(self.get_parameter("stable_frames_required").value)

            cv2.drawContours(dbg, [c], -1, (0, 255, 0), 2)
            cv2.circle(dbg, (int(self._fx), int(self._fy)), 6, (0, 255, 255), -1)
        else:
            self._stable_count = 0
            self._fx = None
            self._fy = None

        cv2.line(dbg, (int(cx_ref), 0), (int(cx_ref), h - 1), (255, 255, 255), 1)
        cv2.line(dbg, (0, int(cy_ref)), (w - 1, int(cy_ref)), (255, 255, 255), 1)

        red_candidates = []
        for rc in red_contours:
            r_area = float(cv2.contourArea(rc))
            if r_area < min_area:
                continue
            rm = cv2.moments(rc)
            if abs(rm["m00"]) < 1e-6:
                continue
            rcx = float(rm["m10"] / rm["m00"])
            rcy = float(rm["m01"] / rm["m00"])
            red_candidates.append({"cx": round(rcx, 2), "cy": round(rcy, 2), "area": round(r_area, 2)})
            cv2.drawContours(dbg, [rc], -1, (0, 80, 255), 1)
        self._last_red.data = status_json(stage="bottom_red", count=len(red_candidates), candidates=red_candidates[:8])

        status = String()
        status.data = status_json(
            stage="bottom",
            ok=True,
            detected=best is not None,
            red_candidates=len(red_candidates),
            aligned=aligned,
            stable_count=self._stable_count,
            confidence=round(float(center_msg.vector.z), 4),
            err_px=round(float(error_msg.vector.z), 3),
            center_x=round(float(center_msg.vector.x), 2),
            center_y=round(float(center_msg.vector.y), 2),
        )
        return center_msg, error_msg, status, dbg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BottomDrumDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

