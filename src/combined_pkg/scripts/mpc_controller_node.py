#!/usr/bin/env python3
import sys
import os
import numpy as np
import rclpy
from std_msgs.msg import String, Bool, Float64MultiArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from controller_base import ControllerNodeBase, N, DT

try:
    from scipy.linalg import solve_discrete_are
except ImportError as e:
    raise SystemExit(
        "scipy is required for MPC. Install with:\n"
        "  sudo apt install python3-scipy\n"
        f"Original error: {e}"
    )

# ---- MATLAB defaults ----
NP = 20
Q_W_DEF = 500.0
R_W_DEF = 0.001
U_MAX = 50.0
U_MIN = -U_MAX

A_d = np.array([[1.0, DT], [0.0, 1.0]])
B_d = np.array([[DT ** 2 / 2.0], [DT]])


def build_mpc_matrices(q_w: float, r_w: float):
    """همان ساخت Sx, Su, H از کد قبلی — فقط Q_W/R_W متغیر."""
    Qy = np.diag([q_w, 0.0])
    P_term = solve_discrete_are(A_d, B_d, Qy, np.array([[r_w]]))
    A_pows = [np.eye(2)]
    for _ in range(NP):
        A_pows.append(A_pows[-1] @ A_d)
    SxFull = np.zeros((2 * NP, 2))
    SuFull = np.zeros((2 * NP, NP))
    for j in range(1, NP + 1):
        rows = slice(2 * (j - 1), 2 * j)
        SxFull[rows, :] = A_pows[j]
        for l in range(1, j + 1):
            SuFull[rows, l - 1:l] = A_pows[j - l] @ B_d
    Wbar = np.zeros((2 * NP, 2 * NP))
    for j in range(1, NP):
        rows = slice(2 * (j - 1), 2 * j)
        Wbar[rows, rows] = Qy
    Wbar[2 * (NP - 1):2 * NP, 2 * (NP - 1):2 * NP] = P_term
    Rbar = r_w * np.eye(NP)
    H_qp = 2.0 * (SuFull.T @ Wbar @ SuFull + Rbar)
    H_qp = 0.5 * (H_qp + H_qp.T)
    H_qp_inv = np.linalg.inv(H_qp)
    return SxFull, SuFull, Wbar, H_qp_inv


class MPCControllerNode(ControllerNodeBase):
    def __init__(self):
        super().__init__("mpc_controller_node", "mpc")
        self.active = False
        self.pso_on = False
        self.q_w = float(Q_W_DEF)
        self.r_w = float(R_W_DEF)
        self.SxFull, self.SuFull, self.Wbar, self.H_qp_inv = build_mpc_matrices(
            self.q_w, self.r_w
        )

        self.create_subscription(String, "/controller/mode", self._mode_cb, 10)
        self.create_subscription(Bool, "/controller/pso_enable", self._pso_cb, 10)
        self.create_subscription(Float64MultiArray, "/controller/gains_mpc", self._gains_cb, 10)
        self.get_logger().info("mpc_controller_node ready (inactive until mode=mpc)")

    def _mode_cb(self, msg: String):
        was = self.active
        self.active = msg.data.strip().lower() == "mpc"
        if self.active and not was:
            self.get_logger().info("MPC ACTIVE")
        elif not self.active and was:
            self.get_logger().info("MPC inactive")

    def _pso_cb(self, msg: Bool):
        self.pso_on = bool(msg.data)
        if not self.pso_on:
            self.q_w, self.r_w = float(Q_W_DEF), float(R_W_DEF)
            self.SxFull, self.SuFull, self.Wbar, self.H_qp_inv = build_mpc_matrices(
                self.q_w, self.r_w
            )
            self.get_logger().info("MPC: PSO OFF → MATLAB Q_W/R_W")

    def _gains_cb(self, msg: Float64MultiArray):
        # 2 values: Q_W, R_W
        if not self.pso_on or len(msg.data) < 2:
            return
        self.q_w = float(max(msg.data[0], 1.0))
        self.r_w = float(max(msg.data[1], 1e-6))
        self.SxFull, self.SuFull, self.Wbar, self.H_qp_inv = build_mpc_matrices(
            self.q_w, self.r_w
        )
        self.get_logger().info(f"MPC: PSO weights Q_W={self.q_w:.3f} R_W={self.r_w:.6f}")

    def solve_joint_mpc(self, q0, dq0, q_des_horizon):
        x0 = np.array([q0, dq0])
        Xref = np.zeros(2 * NP)
        for j in range(1, NP + 1):
            Xref[2 * (j - 1)] = q_des_horizon[j - 1]
        E0 = self.SxFull @ x0 - Xref
        f_qp = 2.0 * (self.SuFull.T @ (self.Wbar @ E0))
        U_unc = -self.H_qp_inv @ f_qp
        U_clipped = np.clip(U_unc, U_MIN, U_MAX)
        return float(U_clipped[0])

    def compute_ddq(self, e, de, q_des, dq_des, ddq_des):
        # همان منطق قبلی
        if not self.active:
            return np.zeros(N)
        q_cmd = e + q_des
        dq_cmd = de + dq_des
        ddq_cmd = np.zeros(N)
        for i in range(N):
            q_des_horizon = np.full(NP, q_des[i])
            ddq_cmd[i] = self.solve_joint_mpc(q_cmd[i], dq_cmd[i], q_des_horizon)
        return ddq_cmd


def main():
    rclpy.init()
    node = MPCControllerNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()