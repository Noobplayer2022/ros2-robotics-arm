#!/usr/bin/env python3
"""
Live plot: joint setpoint (/controller/q_target) vs actual (/joint_states).
Run while main.py + control_panel are running:

    ros2 run combined_pkg live_plot.py

Needs: sudo apt install python3-matplotlib
"""
import sys
import os
import threading
import time
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import matplotlib.animation as animation

WINDOW_SEC = 20.0
UPDATE_MS = 100
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
N = len(JOINT_NAMES)


class LivePlotNode(Node):
    def __init__(self):
        super().__init__("live_plot")
        self.lock = threading.Lock()
        self.t0 = time.time()

        self.latest_sp = None   # np.array N
        self.latest_q = None    # np.array N

        self.t = deque()
        self.sp = [deque() for _ in range(N)]
        self.q = [deque() for _ in range(N)]

        self.create_subscription(
            Float64MultiArray, "/controller/q_target", self._sp_cb, 10)
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.get_logger().info("live_plot ready — waiting for setpoint + joint_states")

    def _sp_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= N:
            with self.lock:
                self.latest_sp = np.array(msg.data[:N], dtype=float)

    def _js_cb(self, msg: JointState):
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
        try:
            q = np.array([name_to_pos[n] for n in JOINT_NAMES], dtype=float)
        except KeyError:
            return
        with self.lock:
            self.latest_q = q

    def sample(self):
        now = time.time() - self.t0
        with self.lock:
            sp = self.latest_sp
            q = self.latest_q
        if sp is None and q is None:
            return

        self.t.append(now)
        for i in range(N):
            self.sp[i].append(sp[i] if sp is not None else float("nan"))
            self.q[i].append(q[i] if q is not None else float("nan"))

        while self.t and (now - self.t[0]) > WINDOW_SEC:
            self.t.popleft()
            for i in range(N):
                self.sp[i].popleft()
                self.q[i].popleft()


def main():
    rclpy.init()
    node = LivePlotNode()

    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    fig, axes = plt.subplots(N, 1, figsize=(10, 10), sharex=True)
    fig.suptitle(
        "Setpoint vs Actual Joint Position (PID / MPC)",
        fontsize=13, fontweight="bold")

    sp_lines, q_lines = [], []
    for ax, name in zip(axes, JOINT_NAMES):
        sp_l, = ax.plot([], [], "r--", lw=1.6, label="Setpoint")
        q_l, = ax.plot([], [], "b-", lw=1.8, label="Actual")
        ax.set_ylabel(name)
        ax.grid(True, linestyle="--", alpha=0.5)
        ax.legend(loc="upper right", fontsize=8)
        sp_lines.append(sp_l)
        q_lines.append(q_l)
    axes[-1].set_xlabel("Time (s)")

    def update(_):
        node.sample()
        if len(node.t) < 2:
            return sp_lines + q_lines
        t = list(node.t)
        for i, ax in enumerate(axes):
            sp_lines[i].set_data(t, list(node.sp[i]))
            q_lines[i].set_data(t, list(node.q[i]))
            ax.set_xlim(max(0, t[-1] - WINDOW_SEC), max(WINDOW_SEC, t[-1]))
            vals = list(node.sp[i]) + list(node.q[i])
            vals = [v for v in vals if v == v]  # drop nan
            if vals:
                ymin, ymax = min(vals), max(vals)
                pad = max(0.05, (ymax - ymin) * 0.15)
                ax.set_ylim(ymin - pad, ymax + pad)
        return sp_lines + q_lines

    ani = animation.FuncAnimation(fig, update, interval=UPDATE_MS, blit=False)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    try:
        plt.show()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()