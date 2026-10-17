#!/usr/bin/env python3
import sys
import os
import numpy as np
import rclpy
from std_msgs.msg import String, Bool, Float64MultiArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from controller_base import ControllerNodeBase, N, DT

# ---- MATLAB defaults (بدون PSO) ----
KP_MATLAB = np.array([150.0, 250.0, 200.0, 150.0, 100.0])
KD_MATLAB = np.array([20.0, 30.0, 25.0, 18.0, 12.0])
KI_MATLAB = np.array([5.0, 8.0, 6.0, 4.0, 3.0])
GAIN_SCALE = 0.02
KP_DEF = KP_MATLAB * GAIN_SCALE
KD_DEF = KD_MATLAB * GAIN_SCALE
KI_DEF = KI_MATLAB * GAIN_SCALE
E_INT_MAX = 2.0


class PIDControllerNode(ControllerNodeBase):
    def __init__(self):
        super().__init__("pid_controller_node", "pid")
        self.e_int = np.zeros(N)
        self.kp = KP_DEF.copy()
        self.kd = KD_DEF.copy()
        self.ki = KI_DEF.copy()
        self.pso_on = False
        self.active = True

        self.create_subscription(String, "/controller/mode", self._mode_cb, 10)
        self.create_subscription(Bool, "/controller/pso_enable", self._pso_cb, 10)
        self.create_subscription(Float64MultiArray, "/controller/gains_pid", self._gains_cb, 10)
        self.get_logger().info("pid_controller_node ready")

    def _mode_cb(self, msg: String):
        was = self.active
        self.active = msg.data.strip().lower() in ("pid", "")
        if self.active and not was:
            self.e_int[:] = 0.0
            self.get_logger().info("PID ACTIVE")
        elif not self.active and was:
            self.get_logger().info("PID inactive")

    def _pso_cb(self, msg: Bool):
        self.pso_on = bool(msg.data)
        if not self.pso_on:
            self.kp, self.kd, self.ki = KP_DEF.copy(), KD_DEF.copy(), KI_DEF.copy()
            self.get_logger().info("PID: PSO OFF → MATLAB default gains")

    def _gains_cb(self, msg: Float64MultiArray):
        # 15 values: Kp(5) Kd(5) Ki(5) — فقط اگر PSO روشن باشد
        if not self.pso_on or len(msg.data) < 15:
            return
        d = np.asarray(msg.data[:15], dtype=float)
        self.kp, self.kd, self.ki = d[0:5], d[5:10], d[10:15]
        self.e_int[:] = 0.0
        self.get_logger().info(f"PID: PSO gains applied Kp={np.round(self.kp, 3)}")

    def compute_ddq(self, e, de, q_des, dq_des, ddq_des):
        # همان فرمول قبلی
        if not self.active:
            return np.zeros(N)
        self.e_int = np.clip(self.e_int + e * DT, -E_INT_MAX, E_INT_MAX)
        return -self.kp * e - self.kd * de - self.ki * self.e_int


def main():
    rclpy.init()
    node = PIDControllerNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()