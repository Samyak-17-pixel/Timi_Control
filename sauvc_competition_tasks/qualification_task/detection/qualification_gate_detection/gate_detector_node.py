from __future__ import annotations

import math
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from qualification_gate_interfaces.msg import GateDetection
from rclpy.node import Node
from sensor_msgs.msg import Image


class GateDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("gate_detector")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("gate_topic", "/gate/detection")
        self.declare_parameter("debug_image_topic", "/gate/debug_image")
        self.declare_parameter("mask_topic", "/gate/mask")
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("publish_mask_image", True)
        self.declare_parameter("publish_rate_hz", 10.0)

        self.declare_parameter("use_color_boost", True)
        self.declare_parameter("hsv_low1", [0, 70, 60])
        self.declare_parameter("hsv_high1", [15, 255, 255])
        self.declare_parameter("hsv_low2", [160, 70, 60])
        self.declare_parameter("hsv_high2", [180, 255, 255])
        self.declare_parameter("mask_open_kernel", 3)
        self.declare_parameter("mask_close_kernel", 7)
        self.declare_parameter("mask_blur_kernel", 5)

        self.declare_parameter("blur_kernel", 5)
        self.declare_parameter("canny_low", 45)
        self.declare_parameter("canny_high", 130)
        self.declare_parameter("hough_threshold", 45)
        self.declare_parameter("hough_min_line_length", 90)
        self.declare_parameter("hough_max_line_gap", 18)

        self.declare_parameter("roi_top_frac", 0.12)
        self.declare_parameter("max_vertical_tilt_deg", 14.0)
        self.declare_parameter("min_vertical_length_px", 130.0)
        self.declare_parameter("min_y_overlap_px", 90.0)
        self.declare_parameter("min_pole_separation_px", 80.0)
        self.declare_parameter("max_pole_separation_frac", 0.75)
        self.declare_parameter("center_bias_weight", 0.08)

        self.declare_parameter("stable_center_frames", 3)

        self._bridge = CvBridge()
        self._valid_center_streak = 0
        self._last_det = GateDetection()
        self._last_det.gate_center_x_px = float("nan")
        self._last_det.alignment_status = 0
        self._last_image_time = self.get_clock().now()
        self._last_no_image_warn_time = self.get_clock().now()

        image_topic = str(self.get_parameter("image_topic").value)
        gate_topic = str(self.get_parameter("gate_topic").value)
        debug_topic = str(self.get_parameter("debug_image_topic").value)
        mask_topic = str(self.get_parameter("mask_topic").value)

        self._pub = self.create_publisher(GateDetection, gate_topic, 10)
        self._pub_dbg = self.create_publisher(Image, debug_topic, 10)
        self._pub_mask = self.create_publisher(Image, mask_topic, 10)
        self.create_subscription(Image, image_topic, self._on_image, 10)
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(1.0, rate_hz), self._publish_detection_heartbeat)
        self.get_logger().info(f"gate_detector | image={image_topic} out={gate_topic}")

    def _on_image(self, msg: Image) -> None:
        self._last_image_time = self.get_clock().now()
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            return

        det_msg, dbg, mask = self._detect(frame)
        self._last_det = det_msg
        self._pub.publish(self._last_det)
        if bool(self.get_parameter("publish_debug_image").value):
            self._pub_dbg.publish(self._bridge.cv2_to_imgmsg(dbg, encoding="bgr8"))
        if bool(self.get_parameter("publish_mask_image").value):
            self._pub_mask.publish(self._bridge.cv2_to_imgmsg(mask, encoding="mono8"))

    def _publish_detection_heartbeat(self) -> None:
        self._pub.publish(self._last_det)
        now = self.get_clock().now()
        dt = (now - self._last_image_time).nanoseconds * 1e-9
        dt_warn = (now - self._last_no_image_warn_time).nanoseconds * 1e-9
        if dt > 2.0 and dt_warn > 5.0:
            self._last_no_image_warn_time = now
            self.get_logger().warn(
                "No camera frames received on image_topic for %.1fs. "
                "Check image_topic and camera publisher." % dt
            )

    def _parse_hsv_triplet(self, name: str, fallback: list[int]) -> np.ndarray:
        value = self.get_parameter(name).value
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            value = fallback
        triplet = [int(v) for v in value]
        return np.array(triplet, dtype=np.uint8)

    def _color_mask(self, frame_bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        low1 = self._parse_hsv_triplet("hsv_low1", [0, 70, 60])
        high1 = self._parse_hsv_triplet("hsv_high1", [15, 255, 255])
        low2 = self._parse_hsv_triplet("hsv_low2", [160, 70, 60])
        high2 = self._parse_hsv_triplet("hsv_high2", [180, 255, 255])

        m1 = cv2.inRange(hsv, low1, high1)
        m2 = cv2.inRange(hsv, low2, high2)
        mask = cv2.bitwise_or(m1, m2)

        k_open = max(1, int(self.get_parameter("mask_open_kernel").value))
        k_close = max(1, int(self.get_parameter("mask_close_kernel").value))
        if k_open % 2 == 0:
            k_open += 1
        if k_close % 2 == 0:
            k_close += 1
        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_open, k_open))
        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_close, k_close))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)

        k_blur = max(1, int(self.get_parameter("mask_blur_kernel").value))
        if k_blur % 2 == 0:
            k_blur += 1
        if k_blur > 1:
            mask = cv2.GaussianBlur(mask, (k_blur, k_blur), 0.0)
            _, mask = cv2.threshold(mask, 50, 255, cv2.THRESH_BINARY)
        return mask

    def _detect(self, frame_bgr: np.ndarray) -> tuple[GateDetection, np.ndarray, np.ndarray]:
        h, w = frame_bgr.shape[:2]
        roi_top = int(float(self.get_parameter("roi_top_frac").value) * h)
        roi_top = max(0, min(h - 1, roi_top))

        if bool(self.get_parameter("use_color_boost").value):
            proc = self._color_mask(frame_bgr)
        else:
            gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
            k = int(self.get_parameter("blur_kernel").value)
            if k % 2 == 0:
                k += 1
            k = max(3, k)
            proc = cv2.GaussianBlur(gray, (k, k), 0.0)

        edges = cv2.Canny(
            proc,
            threshold1=int(self.get_parameter("canny_low").value),
            threshold2=int(self.get_parameter("canny_high").value),
        )
        edges[:roi_top, :] = 0

        lines = cv2.HoughLinesP(
            edges,
            rho=1.0,
            theta=np.pi / 180.0,
            threshold=int(self.get_parameter("hough_threshold").value),
            minLineLength=int(self.get_parameter("hough_min_line_length").value),
            maxLineGap=int(self.get_parameter("hough_max_line_gap").value),
        )

        verticals: list[dict] = []
        max_tilt = math.radians(float(self.get_parameter("max_vertical_tilt_deg").value))
        min_len = float(self.get_parameter("min_vertical_length_px").value)
        tan_lim = math.tan(max_tilt)

        if lines is not None:
            for ln in lines[:, 0]:
                x1, y1, x2, y2 = [int(v) for v in ln]
                dx = float(x2 - x1)
                dy = float(y2 - y1)
                abs_dy = abs(dy)
                if abs_dy < 1.0:
                    continue
                if abs(dx) > tan_lim * abs_dy:
                    continue
                length = math.hypot(dx, dy)
                if length < min_len:
                    continue
                y_top = min(y1, y2)
                y_bot = max(y1, y2)
                if y_bot <= roi_top:
                    continue
                verticals.append(
                    {
                        "x": 0.5 * (x1 + x2),
                        "y_top": y_top,
                        "y_bot": y_bot,
                        "length": length,
                        "line": (x1, y1, x2, y2),
                    }
                )

        pair = self._best_pair(verticals, w)
        out = GateDetection()
        out.alignment_status = 0
        out.gate_center_x_px = float("nan")
        out.center_error_px = 0

        dbg = frame_bgr.copy()
        for v in verticals:
            x1, y1, x2, y2 = v["line"]
            cv2.line(dbg, (x1, y1), (x2, y2), (140, 140, 140), 1, cv2.LINE_AA)

        if pair is not None:
            left, right = pair
            gate_center = 0.5 * (left["x"] + right["x"])
            center_err = int(round(gate_center - (0.5 * w)))
            out.pole1_detected = True
            out.pole2_detected = True

            self._valid_center_streak += 1
            stable_need = int(self.get_parameter("stable_center_frames").value)
            if self._valid_center_streak >= max(1, stable_need):
                out.gate_center_valid = True
                out.gate_center_x_px = float(gate_center)
                out.center_error_px = center_err
                out.alignment_status = 1 if abs(center_err) <= 8 else 2
            else:
                out.gate_center_valid = False

            for c in (left, right):
                x1, y1, x2, y2 = c["line"]
                cv2.line(dbg, (x1, y1), (x2, y2), (0, 255, 0), 2, cv2.LINE_AA)
            cv2.line(dbg, (int(round(gate_center)), 0), (int(round(gate_center)), h - 1), (0, 255, 255), 1)
        else:
            self._valid_center_streak = 0
            out.gate_center_valid = False
            if verticals:
                strongest = max(verticals, key=lambda v: v["length"])
                if strongest["x"] < 0.5 * w:
                    out.pole1_detected = True
                    out.pole2_detected = False
                else:
                    out.pole1_detected = False
                    out.pole2_detected = True
                x1, y1, x2, y2 = strongest["line"]
                cv2.line(dbg, (x1, y1), (x2, y2), (0, 165, 255), 2, cv2.LINE_AA)
            else:
                out.pole1_detected = False
                out.pole2_detected = False

        cv2.line(dbg, (w // 2, 0), (w // 2, h - 1), (255, 0, 0), 1)
        return out, dbg, proc

    def _best_pair(self, verticals: list[dict], width: int) -> Optional[tuple[dict, dict]]:
        if len(verticals) < 2:
            return None
        min_sep = float(self.get_parameter("min_pole_separation_px").value)
        max_sep = float(self.get_parameter("max_pole_separation_frac").value) * float(width)
        min_ov = float(self.get_parameter("min_y_overlap_px").value)
        center_w = float(self.get_parameter("center_bias_weight").value)
        img_c = 0.5 * float(width)

        best: Optional[tuple[dict, dict]] = None
        best_score = -1e9
        sorted_vs = sorted(verticals, key=lambda v: v["length"], reverse=True)[:24]
        n = len(sorted_vs)
        for i in range(n):
            a = sorted_vs[i]
            for j in range(i + 1, n):
                b = sorted_vs[j]
                sep = abs(a["x"] - b["x"])
                if sep < min_sep or sep > max_sep:
                    continue
                ov = min(a["y_bot"], b["y_bot"]) - max(a["y_top"], b["y_top"])
                if ov < min_ov:
                    continue
                center = 0.5 * (a["x"] + b["x"])
                score = (a["length"] + b["length"]) + 0.4 * ov - center_w * abs(center - img_c)
                if score > best_score:
                    left, right = (a, b) if a["x"] <= b["x"] else (b, a)
                    best = (left, right)
                    best_score = score
        return best


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GateDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

