#!/usr/bin/env python3
"""Cube pose only (fixed /cube/pose). No camera."""
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String
import tf2_ros
from tf2_geometry_msgs import do_transform_point

import config


class CubeDetector(Node):
    def __init__(self):
        super().__init__("cube_detector")
        self.target_frame = config.BASE_LINK
        self.history = []
        self.state_lock = threading.Lock()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.debug_pub = self.create_publisher(PoseStamped, "/cube/pose_base", 10)

        self.create_subscription(
            PoseStamped, config.CUBE_POSE_TOPIC, self._cube_cb, 10
        )
        self.create_subscription(
            String, config.ENABLED_COLORS_TOPIC, lambda _m: None, 1
        )
        self.create_timer(0.25, self._fixed_timer)
        self.get_logger().info("CubeDetector: pose-only mode (no camera)")

    def reset_tracking(self):
        with self.state_lock:
            self.history.clear()

    def _world_to_base(self, x, y, z):
        pt = PointStamped()
        pt.header.frame_id = "world"
        pt.header.stamp = rclpy.time.Time().to_msg()
        pt.point.x = float(x)
        pt.point.y = float(y)
        pt.point.z = float(z)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                "world",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.3),
            )
            out = do_transform_point(pt, tf)
            return out.point.x, out.point.y, out.point.z
        except Exception:
            # URDF: base ≈ (-0.35, -0.10, 0) در world
            return x + 0.35, y + 0.10, z

    def _push(self, x, y, z):
        t = self.get_clock().now().nanoseconds * 1e-9
        with self.state_lock:
            self.history.append((t, x, y, z))
            if len(self.history) > 20:
                self.history.pop(0)
        msg = PoseStamped()
        msg.header.frame_id = self.target_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.debug_pub.publish(msg)

    def _cube_cb(self, msg: PoseStamped):
        bx, by, bz = self._world_to_base(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )
        self._push(bx, by, bz)

    def _fixed_timer(self):
        if not getattr(config, "USE_FIXED_CUBE_POSE", True):
            return
        x, y, z = config.CUBE_WORLD_XYZ
        self._push(*self._world_to_base(x, y, z))

    def predict_setpoint(self, lead_time=0.0):
        with self.state_lock:
            if not self.history:
                return None
            _, x, y, z = self.history[-1]
        return float(x), float(y), float(z)