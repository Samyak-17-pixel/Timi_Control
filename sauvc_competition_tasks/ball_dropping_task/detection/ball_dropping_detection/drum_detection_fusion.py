from __future__ import annotations

import json
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import String

from .common import status_json


class DrumDetectionFusion(Node):
    def __init__(self) -> None:
        super().__init__("drum_detection_fusion")

        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("prefer_bottom_min_conf", 0.25)
        self.declare_parameter("front_min_conf", 0.20)
        self.declare_parameter("bottom_timeout_sec", 1.0)
        self.declare_parameter("front_timeout_sec", 1.0)

        self._front = Vector3Stamped()
        self._bottom = Vector3Stamped()
        self._front_status = {}
        self._bottom_status = {}
        self._front_t = 0.0
        self._bottom_t = 0.0

        self._pub_target = self.create_publisher(Vector3Stamped, "/ball_dropping/fusion/target", 10)
        self._pub_state = self.create_publisher(String, "/ball_dropping/fusion/state", 10)
        self._pub_status = self.create_publisher(String, "/ball_dropping/fusion/status", 10)

        self.create_subscription(Vector3Stamped, "/ball_dropping/front/blue_target", self._on_front, 10)
        self.create_subscription(Vector3Stamped, "/ball_dropping/bottom/center", self._on_bottom, 10)
        self.create_subscription(String, "/ball_dropping/front/status", self._on_front_status, 10)
        self.create_subscription(String, "/ball_dropping/bottom/status", self._on_bottom_status, 10)

        hz = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(1.0, hz), self._on_timer)
        self.get_logger().info("drum_detection_fusion started")

    def _on_front(self, msg: Vector3Stamped) -> None:
        self._front = msg
        self._front_t = time.time()

    def _on_bottom(self, msg: Vector3Stamped) -> None:
        self._bottom = msg
        self._bottom_t = time.time()

    def _on_front_status(self, msg: String) -> None:
        try:
            self._front_status = json.loads(msg.data)
        except Exception:
            self._front_status = {"parse_error": True}

    def _on_bottom_status(self, msg: String) -> None:
        try:
            self._bottom_status = json.loads(msg.data)
        except Exception:
            self._bottom_status = {"parse_error": True}

    def _on_timer(self) -> None:
        now = time.time()
        front_age = now - self._front_t
        bottom_age = now - self._bottom_t
        front_ok = front_age <= float(self.get_parameter("front_timeout_sec").value)
        bottom_ok = bottom_age <= float(self.get_parameter("bottom_timeout_sec").value)
        front_conf = float(self._front.vector.z)
        bottom_conf = float(self._bottom.vector.z)

        sel = "none"
        out = Vector3Stamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.vector.x = -1.0
        out.vector.y = -1.0
        out.vector.z = 0.0

        if bottom_ok and bottom_conf >= float(self.get_parameter("prefer_bottom_min_conf").value):
            sel = "bottom"
            out.vector.x = float(self._bottom.vector.x)
            out.vector.y = float(self._bottom.vector.y)
            out.vector.z = float(bottom_conf)
        elif front_ok and front_conf >= float(self.get_parameter("front_min_conf").value):
            sel = "front"
            out.vector.x = float(self._front.vector.x)
            out.vector.y = float(self._front.vector.y)
            out.vector.z = float(front_conf)

        state = String()
        state.data = status_json(
            selected=sel,
            front_conf=round(front_conf, 4),
            bottom_conf=round(bottom_conf, 4),
            front_age=round(front_age, 3),
            bottom_age=round(bottom_age, 3),
        )

        status = String()
        status.data = status_json(
            stage="fusion",
            selected=sel,
            target_x=round(float(out.vector.x), 2),
            target_y=round(float(out.vector.y), 2),
            confidence=round(float(out.vector.z), 4),
            front=self._front_status,
            bottom=self._bottom_status,
        )

        self._pub_target.publish(out)
        self._pub_state.publish(state)
        self._pub_status.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DrumDetectionFusion()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

