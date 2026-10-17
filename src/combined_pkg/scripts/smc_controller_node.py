#!/usr/bin/env python3
"""Sliding Mode Controller (ROS 2) — New_SMC_Controller_Lyapunov.m
    s = de + lambda * e
    ddq_cmd = ddq_des - lambda * de - eta * tanh(s / phi)
"""
import sys
import os
import numpy as np
import rclpy
from std_msgs.msg import String, Bool, Float64MultiArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from controller_base import ControllerNodeBase, N

# ---- MATLAB defaults ----
LAMBDA_MATLAB = 25.0
ETA_MATLAB = np.array([10.0, 15.0, 12.0, 10.0, 8.0])
PHI_DEF = 0.1
GAIN_SCALE = 0.05
LAMBDA_DEF = LAMBDA_MATLAB * GAIN_SCALE
ETA_DEF = ETA_MATLAB * GAIN_SCALE


class SMCControllerNode(ControllerNodeBase):
    def __init__(self):
        super().__init__("smc_controller_node", "smc")
        self.lam = float(LAMBDA_DEF)
        self.eta = ETA_DEF.copy()
        self.phi = float(PHI_DEF)
        self.pso_on = False
        self.active = False

        self.create_subscription(String, "/controller/mode", self._mode_cb, 10)
        self.create_subscription(Bool, "/controller/pso_enable", self._pso_cb, 10)
        self.create_subscription(Float64MultiArray, "/controller/gains_smc", self._gains_cb, 10)
        self.get_logger().info("smc_controller_node ready")

    def _mode_cb(self, msg: String):
        was = self.active
        self.active = msg.data.strip().lower() == "smc"
        if self.active and not was:
            self.get_logger().info("SMC ACTIVE")
        elif not self.active and was:
            self.get_logger().info("SMC inactive")

    def _pso_cb(self, msg: Bool):
        self.pso_on = bool(msg.data)
        if not self.pso_on:
            self.lam = float(LAMBDA_DEF)
            self.eta = ETA_DEF.copy()
            self.phi = float(PHI_DEF)
            self.get_logger().info("SMC: PSO OFF → MATLAB default gains")

    def _gains_cb(self, msg: Float64MultiArray):
        # 7 values: lambda(1) + eta(5) + phi(1)
        if not self.pso_on or len(msg.data) < 7:
            return
        d = np.asarray(msg.data[:7], dtype=float)
        self.lam = float(d[0])
        self.eta = d[1:6].copy()
        self.phi = float(max(d[6], 1e-4))
        self.get_logger().info(f"SMC: PSO gains lam={self.lam:.3f} phi={self.phi:.3f}")

    def compute_ddq(self, e, de, q_des, dq_des, ddq_des):
        # همان فرمول MATLAB
        if not self.active:
            return np.zeros(N)
        s = de + self.lam * e
        return ddq_des - self.lam * de - self.eta * np.tanh(s / self.phi)


def main():
    rclpy.init()
    node = SMCControllerNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()