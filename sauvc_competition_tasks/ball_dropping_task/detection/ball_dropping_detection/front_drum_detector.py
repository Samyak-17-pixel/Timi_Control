from __future__ import annotations

import time
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String

from .common import clamp, status_json


class FrontDrumDetector(Node):
    def __init__(self) -> None:
        super().__init__("front_drum_detector")

        self.declare_parameter("image_topic", "/front/image_raw")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("image_width", 640.0)
        self.declare_parameter("image_height", 480.0)

        self.declare_parameter("blur_kernel", 5)
        self.declare_parameter("canny_low", 40)
        self.declare_parameter("canny_high", 120)
        self.declare_parameter("contour_min_area", 900.0)
        self.declare_parameter("contour_max_area_frac", 0.65)
        self.declare_parameter("candidate_min_w", 28.0)
        self.declare_parameter("candidate_min_h", 18.0)
        self.declare_parameter("candidate_min_aspect", 0.45)
        self.declare_parameter("candidate_max_aspect", 3.2)

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
        self.declare_parameter("min_blue_score", 0.08)
        self.declare_parameter("min_red_score", 0.08)
        self.declare_parameter("contrast_ring_scale", 1.35)
        self.declare_parameter("color_contrast_weight", 2.2)
        self.declare_parameter("color_purity_weight", 1.1)
        self.declare_parameter("scene_expected_blue", 1)
        self.declare_parameter("scene_expected_red", 3)
        self.declare_parameter("scene_count_penalty", 0.06)

        self.declare_parameter("distance_px_ref", 220.0)
        self.declare_parameter("distance_m_ref", 1.5)
        self.declare_parameter("distance_power", 1.0)

        self.declare_parameter("lock_confidence_drop", 0.35)
        self.declare_parameter("lock_timeout_sec", 2.0)

        self._bridge = CvBridge()
        self._last_center = Vector3Stamped()
        self._last_dist = Float32()
        self._last_status = String()
        self._last_candidates = String()
        self._last_debug: Optional[np.ndarray] = None
        self._last_image_time = 0.0
        self._lock_x: Optional[float] = None
        self._lock_y: Optional[float] = None
        self._lock_t = 0.0

        self._pub_center = self.create_publisher(Vector3Stamped, "/ball_dropping/front/blue_target", 10)
        self._pub_dist = self.create_publisher(Float32, "/ball_dropping/front/blue_distance_m", 10)
        self._pub_status = self.create_publisher(String, "/ball_dropping/front/status", 10)
        self._pub_candidates = self.create_publisher(String, "/ball_dropping/front/candidates", 10)
        self._pub_debug = self.create_publisher(Image, "/ball_dropping/front/debug_image", 10)

        image_topic = str(self.get_parameter("image_topic").value)
        self.create_subscription(Image, image_topic, self._on_image, 10)
        hz = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(1.0, hz), self._on_publish_timer)
        self.get_logger().info(f"front_drum_detector | image={image_topic}")

    def _on_image(self, msg: Image) -> None:
        self._last_image_time = time.time()
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self._last_status.data = status_json(stage="front", ok=False, reason=f"cv_bridge_error:{exc}")
            return

        center_msg, dist_msg, cand_msg, status_msg, debug = self._detect(frame)
        self._last_center = center_msg
        self._last_dist = dist_msg
        self._last_candidates = cand_msg
        self._last_status = status_msg
        self._last_debug = debug

    def _on_publish_timer(self) -> None:
        self._pub_center.publish(self._last_center)
        self._pub_dist.publish(self._last_dist)
        self._pub_status.publish(self._last_status)
        self._pub_candidates.publish(self._last_candidates)

        if bool(self.get_parameter("publish_debug_image").value) and self._last_debug is not None:
            self._pub_debug.publish(self._bridge.cv2_to_imgmsg(self._last_debug, encoding="bgr8"))

        dt = time.time() - self._last_image_time
        if self._last_image_time > 0.0 and dt > 2.0:
            self._last_status.data = status_json(stage="front", ok=False, reason="no_image", age_sec=round(dt, 2))

    def _detect(self, frame_bgr: np.ndarray) -> tuple[Vector3Stamped, Float32, String, String, np.ndarray]:
        h, w = frame_bgr.shape[:2]
        area_img = float(h * w)

        blur_k = int(self.get_parameter("blur_kernel").value)
        if blur_k % 2 == 0:
            blur_k += 1
        blur_k = max(3, blur_k)

        blurred = cv2.GaussianBlur(frame_bgr, (blur_k, blur_k), 0.0)
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(
            gray,
            threshold1=int(self.get_parameter("canny_low").value),
            threshold2=int(self.get_parameter("canny_high").value),
        )

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates: list[dict[str, Any]] = []

        min_area = float(self.get_parameter("contour_min_area").value)
        max_area = float(self.get_parameter("contour_max_area_frac").value) * area_img
        min_w = float(self.get_parameter("candidate_min_w").value)
        min_h = float(self.get_parameter("candidate_min_h").value)
        min_as = float(self.get_parameter("candidate_min_aspect").value)
        max_as = float(self.get_parameter("candidate_max_aspect").value)

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        bh_lo = int(self.get_parameter("blue_h_low").value)
        bh_hi = int(self.get_parameter("blue_h_high").value)
        bs_lo = int(self.get_parameter("blue_s_low").value)
        bv_lo = int(self.get_parameter("blue_v_low").value)
        r1_lo = int(self.get_parameter("red1_h_low").value)
        r1_hi = int(self.get_parameter("red1_h_high").value)
        r2_lo = int(self.get_parameter("red2_h_low").value)
        r2_hi = int(self.get_parameter("red2_h_high").value)
        rs_lo = int(self.get_parameter("red_s_low").value)
        rv_lo = int(self.get_parameter("red_v_low").value)
        ring_scale = float(self.get_parameter("contrast_ring_scale").value)
        w_contrast = float(self.get_parameter("color_contrast_weight").value)
        w_purity = float(self.get_parameter("color_purity_weight").value)

        dbg = frame_bgr.copy()
        cv2.line(dbg, (w // 2, 0), (w // 2, h - 1), (255, 255, 255), 1)

        for c in contours:
            area = float(cv2.contourArea(c))
            if area < min_area or area > max_area:
                continue
            x, y, ww, hh = cv2.boundingRect(c)
            if ww < min_w or hh < min_h:
                continue
            aspect = ww / max(1.0, float(hh))
            if aspect < min_as or aspect > max_as:
                continue

            roi = hsv[y : y + hh, x : x + ww]
            if roi.size == 0:
                continue

            blue_mask = cv2.inRange(roi, (bh_lo, bs_lo, bv_lo), (bh_hi, 255, 255))
            red_mask_1 = cv2.inRange(roi, (r1_lo, rs_lo, rv_lo), (r1_hi, 255, 255))
            red_mask_2 = cv2.inRange(roi, (r2_lo, rs_lo, rv_lo), (r2_hi, 255, 255))
            red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)
            roi_px = float(roi.shape[0] * roi.shape[1])
            blue_ratio = float(np.count_nonzero(blue_mask)) / roi_px
            red_ratio = float(np.count_nonzero(red_mask)) / roi_px

            x0 = max(0, int(x - 0.5 * (ring_scale - 1.0) * ww))
            y0 = max(0, int(y - 0.5 * (ring_scale - 1.0) * hh))
            x1 = min(w, int(x + ww + 0.5 * (ring_scale - 1.0) * ww))
            y1 = min(h, int(y + hh + 0.5 * (ring_scale - 1.0) * hh))
            ring_roi = hsv[y0:y1, x0:x1]
            ring_blue_ratio = 0.0
            ring_red_ratio = 0.0
            if ring_roi.size > 0:
                ring_blue = cv2.inRange(ring_roi, (bh_lo, bs_lo, bv_lo), (bh_hi, 255, 255))
                ring_red_1 = cv2.inRange(ring_roi, (r1_lo, rs_lo, rv_lo), (r1_hi, 255, 255))
                ring_red_2 = cv2.inRange(ring_roi, (r2_lo, rs_lo, rv_lo), (r2_hi, 255, 255))
                ring_red = cv2.bitwise_or(ring_red_1, ring_red_2)
                ring_px = float(ring_roi.shape[0] * ring_roi.shape[1])
                ring_blue_ratio = float(np.count_nonzero(ring_blue)) / max(1.0, ring_px)
                ring_red_ratio = float(np.count_nonzero(ring_red)) / max(1.0, ring_px)

            blue_contrast = blue_ratio - ring_blue_ratio
            red_contrast = red_ratio - ring_red_ratio
            blue_score = w_contrast * blue_contrast + w_purity * blue_ratio
            red_score = w_contrast * red_contrast + w_purity * red_ratio
            cls = "UNKNOWN"
            if blue_score > red_score:
                cls = "BLUE"
            elif red_score > blue_score:
                cls = "RED"
            cx = float(x + ww * 0.5)
            cy = float(y + hh * 0.5)

            candidates.append(
                {
                    "x": float(x),
                    "y": float(y),
                    "w": float(ww),
                    "h": float(hh),
                    "cx": cx,
                    "cy": cy,
                    "area": area,
                    "blue_ratio": blue_ratio,
                    "red_ratio": red_ratio,
                    "blue_contrast": blue_contrast,
                    "red_contrast": red_contrast,
                    "blue_score": blue_score,
                    "red_score": red_score,
                    "class": cls,
                }
            )

        candidates.sort(key=lambda c: max(c["blue_score"], c["red_score"]), reverse=True)
        top = candidates[:8]

        center_msg = Vector3Stamped()
        center_msg.header.stamp = self.get_clock().now().to_msg()
        center_msg.vector.x = -1.0
        center_msg.vector.y = -1.0
        center_msg.vector.z = 0.0

        dist_msg = Float32()
        dist_msg.data = -1.0
        cand_msg = String()
        status_msg = String()

        selected: Optional[dict[str, Any]] = None
        min_blue_score = float(self.get_parameter("min_blue_score").value)
        min_red_score = float(self.get_parameter("min_red_score").value)
        blue_candidates = [c for c in top if c["blue_score"] >= min_blue_score and c["blue_score"] > c["red_score"]]
        red_candidates = [c for c in top if c["red_score"] >= min_red_score and c["red_score"] > c["blue_score"]]
        if blue_candidates:
            selected = max(blue_candidates, key=lambda c: c["blue_score"])

        now = time.time()
        if selected is not None:
            self._lock_x = selected["cx"]
            self._lock_y = selected["cy"]
            self._lock_t = now
        elif self._lock_x is not None and (now - self._lock_t) <= float(self.get_parameter("lock_timeout_sec").value):
            selected = {
                "cx": self._lock_x,
                "cy": self._lock_y,
                "w": 1.0,
                "h": 1.0,
                "blue_ratio": 0.0,
                "score": 0.0,
            }

        for c in top:
            x, y, ww, hh = int(c["x"]), int(c["y"]), int(c["w"]), int(c["h"])
            cv2.rectangle(dbg, (x, y), (x + ww, y + hh), (100, 180, 100), 1)

        if selected is not None:
            cx = float(selected["cx"])
            cy = float(selected["cy"])
            ww = max(1.0, float(selected["w"]))
            center_msg.vector.x = cx
            center_msg.vector.y = cy
            base_conf = clamp(float(selected.get("blue_score", 0.0)), 0.0, 1.0)
            exp_b = int(self.get_parameter("scene_expected_blue").value)
            exp_r = int(self.get_parameter("scene_expected_red").value)
            pen = float(self.get_parameter("scene_count_penalty").value)
            scene_penalty = pen * abs(len(blue_candidates) - exp_b) + pen * abs(len(red_candidates) - exp_r)
            center_msg.vector.z = clamp(base_conf - scene_penalty, 0.0, 1.0)
            px_ref = float(self.get_parameter("distance_px_ref").value)
            m_ref = float(self.get_parameter("distance_m_ref").value)
            pwr = float(self.get_parameter("distance_power").value)
            dist_msg.data = float(m_ref * (px_ref / ww) ** pwr)
            cv2.circle(dbg, (int(cx), int(cy)), 6, (0, 255, 255), -1)
            cv2.putText(dbg, "BLUE_TARGET", (int(cx) - 40, int(cy) - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        status_msg.data = status_json(
            stage="front",
            ok=True,
            image_w=w,
            image_h=h,
            candidates=len(top),
            blue_candidates=len(blue_candidates),
            red_candidates=len(red_candidates),
            selected=selected is not None,
            confidence=round(float(center_msg.vector.z), 4),
            distance_m=round(float(dist_msg.data), 4),
            lock_age_sec=round(now - self._lock_t, 3) if self._lock_t > 0.0 else -1.0,
        )
        cand_msg.data = status_json(stage="front_candidates", candidates=top)
        return center_msg, dist_msg, cand_msg, status_msg, dbg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontDrumDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

