#!/usr/bin/env python3
"""
Spawn cube on conveyor + virtual belt motion + respawn if fallen.
"""
import os
import re
import time
import subprocess
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float64, Bool
from visualization_msgs.msg import Marker

from geometry_msgs.msg import PoseStamped

# --- موقعیت اولیه روی نوار (وسط) ---
SPAWN_X = -0.35
SPAWN_Y = -0.70
SPAWN_Z = 0.10

CUBE_NAME = "pick_cube"
WORLD = "empty"
SAFE_Z = 0.03

# محور حرکت نوار در world (طبق جهت نوار تو)
# اگر اشتباه بود علامت را عوض کن: AXIS_Y = -1.0
AXIS_X = 0.0
AXIS_Y = 1.0

BELT_LEN = 0.45          # نصف طول تقریبی مسیر روی نوار
DEFAULT_SPEED = 0.15     # m/s
RATE_HZ = 10.0


class CubeSpawner(Node):
    def __init__(self):
        super().__init__("cube_spawner")
        share = get_package_share_directory("combined_pkg")
        self.sdf_path = os.path.join(share, "models", "pick_cube", "model.sdf")

        self.x = SPAWN_X
        self.y = SPAWN_Y
        self.z = SPAWN_Z
        self.speed = 0.0          # m/s — از تاپیک می‌آید
        self.belt_on = False

        self.marker_pub = self.create_publisher(Marker, "/pick_cube_marker", 10)
        self.create_subscription(Float64, "/conveyor/cmd_vel", self._vel_cb, 10)
        self.create_subscription(Bool, "/conveyor/enable", self._en_cb, 10)

        self.pose_pub = self.create_publisher(PoseStamped, "/cube/pose", 10)
        self.dt = 1.0 / RATE_HZ
        self.create_timer(self.dt, self._tick)

        self._spawn_at(self.x, self.y, self.z)
        self.get_logger().info(
            "Virtual belt ready. "
            "ros2 topic pub /conveyor/cmd_vel std_msgs/msg/Float64 \"data: 0.15\" --once"
        )

    def _vel_cb(self, msg: Float64):
        self.speed = float(msg.data)
        self.belt_on = abs(self.speed) > 1e-4
        self.get_logger().info(f"belt speed={self.speed:.3f} on={self.belt_on}")

    def _en_cb(self, msg: Bool):
        self.belt_on = bool(msg.data)
        if not self.belt_on:
            self.speed = 0.0

    def _run(self, cmd, timeout=8.0):
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
            return r.returncode == 0, (r.stdout or "") + (r.stderr or "")
        except Exception as e:
            return False, str(e)

    def _remove(self):
        req = f'name: "{CUBE_NAME}", type: MODEL'
        self._run([
            "gz", "service",
            "-s", f"/world/{WORLD}/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1500",
            "--req", req,
        ], timeout=4.0)

    def _spawn_at(self, x, y, z):
        self._remove()
        time.sleep(0.15)
        ok, out = self._run([
            "ros2", "run", "ros_gz_sim", "create",
            "-world", WORLD,
            "-file", self.sdf_path,
            "-name", CUBE_NAME,
            "-x", str(x), "-y", str(y), "-z", str(z),
        ])
        if ok:
            self.x, self.y, self.z = x, y, z
            self.get_logger().info(f"spawn @ ({x:.3f},{y:.3f},{z:.3f})")
        else:
            self.get_logger().error(f"spawn fail: {out[:200]}")

    def _set_pose(self, x, y, z):
        """جابه‌جایی با remove+spawn (ساده و پایدار روی gz)."""
        self._spawn_at(x, y, z)

    def _publish_marker(self):
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "pick_cube"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = self.x
        m.pose.position.y = self.y
        m.pose.position.z = self.z
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.04
        m.color.r, m.color.g, m.color.b, m.color.a = 0.95, 0.15, 0.15, 1.0
        self.marker_pub.publish(m)
        
    def _publish_pose(self):
        p = PoseStamped()
        p.header.frame_id = "world"
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = self.x
        p.pose.position.y = self.y
        p.pose.position.z = self.z
        p.pose.orientation.w = 1.0
        self.pose_pub.publish(p)
    def _tick(self):
        if self.belt_on and abs(self.speed) > 1e-4:
            self.x += AXIS_X * self.speed * self.dt
            self.y += AXIS_Y * self.speed * self.dt
            # برگشت به اول نوار وقتی از انتها رد شد
            dy = self.y - SPAWN_Y
            dx = self.x - SPAWN_X
            if abs(dy) > BELT_LEN or abs(dx) > BELT_LEN:
                self.x, self.y, self.z = SPAWN_X, SPAWN_Y, SPAWN_Z
            self._set_pose(self.x, self.y, self.z)

        self._publish_marker()

        # اگر افتاد
        if self.z < SAFE_Z:
            self.get_logger().warning("cube below SAFE_Z — reset")
            self.x, self.y, self.z = SPAWN_X, SPAWN_Y, SPAWN_Z
            self._spawn_at(self.x, self.y, self.z)


def main():
    rclpy.init()
    node = CubeSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()