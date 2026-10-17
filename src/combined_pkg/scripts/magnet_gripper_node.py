#!/usr/bin/env python3
"""
Magnet on BOTH fingers (link_6 + link_7).
Attach if either finger is within MAX_DISTANCE of the cube.
While attached, cube follows the closer finger.
"""
import math
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import tf2_ros

CUBE_NAME = "pick_cube"
WORLD = "empty"
# فعلاً بزرگ بگذار تا وقتی IK درست شد، بعد کمش کن
MAX_DISTANCE = 0.35
RATE_HZ = 10.0
FINGER_FRAMES = ("link_6", "link_7", "link_5")  # اولویت: دو انگشت، بعد کف

BASE_IN_WORLD = np.array([-0.35, -0.10, 0.0])


class MagnetGripperNode(Node):
    def __init__(self):
        super().__init__("magnet_gripper")
        self.attached = False
        self.want_attach = False
        self.cube_xyz = None  # world

        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=10.0)
        )
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.status_pub = self.create_publisher(Bool, "/magnet/status", 10)
        self.create_subscription(Bool, "/magnet/attach", self._attach_cb, 10)
        self.create_subscription(PoseStamped, "/cube/pose", self._cube_world_cb, 10)
        self.create_subscription(PoseStamped, "/cube/pose_base", self._cube_base_cb, 10)

        self.create_timer(1.0 / RATE_HZ, self._tick)
        self.get_logger().info(
            f"Magnet dual-finger | frames={FINGER_FRAMES} max_d={MAX_DISTANCE}"
        )
        self._publish_status(False)

    def _cube_world_cb(self, msg: PoseStamped):
        self.cube_xyz = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        )

    def _cube_base_cb(self, msg: PoseStamped):
        w = BASE_IN_WORLD + np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])
        self.cube_xyz = (float(w[0]), float(w[1]), float(w[2]))

    def _attach_cb(self, msg: Bool):
        self.want_attach = bool(msg.data)
        if self.want_attach and not self.attached:
            self._try_attach()
        elif not self.want_attach and self.attached:
            self._detach()

    def _publish_status(self, attached: bool):
        self.attached = attached
        m = Bool()
        m.data = attached
        self.status_pub.publish(m)

    def _finger_poses_world(self):
        """لیست (name, x,y,z) برای انگشت‌هایی که TF دارند."""
        out = []
        for frame in FINGER_FRAMES:
            try:
                t = self.tf_buffer.lookup_transform(
                    "world",
                    frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.12),
                )
                p = t.transform.translation
                out.append((frame, p.x, p.y, p.z))
            except Exception:
                continue
        return out

    def _closest_finger(self):
        if self.cube_xyz is None:
            return None, None, 1e9
        best = None
        best_d = 1e9
        for name, x, y, z in self._finger_poses_world():
            d = math.sqrt(
                (x - self.cube_xyz[0]) ** 2
                + (y - self.cube_xyz[1]) ** 2
                + (z - self.cube_xyz[2]) ** 2
            )
            if d < best_d:
                best_d = d
                best = (name, x, y, z)
        return best, best_d

    def _try_attach(self):
        best, d = self._closest_finger()
        if best is None:
            self.get_logger().warning("Magnet: no finger TF / no cube pose")
            self._publish_status(False)
            return
        name, x, y, z = best
        if d > MAX_DISTANCE:
            self.get_logger().warning(
                f"Magnet: too far d={d:.3f} (best={name}) "
                f"finger=({x:.2f},{y:.2f},{z:.2f}) "
                f"cube=({self.cube_xyz[0]:.2f},{self.cube_xyz[1]:.2f},{self.cube_xyz[2]:.2f})"
            )
            self._publish_status(False)
            return
        self._publish_status(True)
        self.get_logger().info(f"Magnet ATTACHED via {name}  d={d:.3f} m")

    def _detach(self):
        self._publish_status(False)
        self.get_logger().info("Magnet DETACHED")

    def _set_cube_pose(self, x, y, z):
        req = (
            f'name: "{CUBE_NAME}", '
            f"position: {{ x: {x:.5f}, y: {y:.5f}, z: {z:.5f} }}, "
            f"orientation: {{ w: 1 }}"
        )
        # بعضی نسخه‌ها Pose با name جدا می‌خواهند
        cmd = [
            "gz", "service",
            "-s", f"/world/{WORLD}/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "300",
            "--req",
            f"name: \"{CUBE_NAME}\", position: {{x: {x:.5f}, y: {y:.5f}, z: {z:.5f}}}, orientation: {{w: 1}}",
        ]
        try:
            subprocess.run(cmd, capture_output=True, text=True, timeout=1.0)
        except Exception:
            pass
        self.cube_xyz = (x, y, z)

    def _tick(self):
        if not self.attached:
            return
        best, d = self._closest_finger()
        if best is None:
            return
        _, x, y, z = best
        self._set_cube_pose(x, y, z)


def main():
    rclpy.init()
    node = MagnetGripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()