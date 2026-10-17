#!/usr/bin/env python3
"""
Automated comparison: PID / SMC / MPC  ×  PSO OFF / ON
- same setpoint for all trials
- strict HOME reset (~10 s) before every trial
- records /joint_states → CSV + PNG
"""
import os
import sys
import time
import csv
import threading
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float64MultiArray
from sensor_msgs.msg import JointState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
N = 5

Q_DES = np.array([-0.386, 1.244, -1.262, 0.0, 1.631])
Q_HOME = np.array([0.0, 0.0, 0.0, 0.0, 0.05])

DURATION = 6.0           # ثبت هر trial
HOME_DURATION = 10.0     # حداکثر زمان برای رسیدن به Home
HOME_TOL = 0.05          # شرط پذیرش Home
HOME_HOLD = 1.0          # بعد از رسیدن، ۱ ثانیه همان‌جا نگه دار
POST_HOME_WAIT = 2.0     # مکث بعد از Home قبل از شروع کنترلر بعدی
SETTLE_AFTER_TRIAL = 1.0
RATE_HZ = 50.0

OUT_DIR = os.path.expanduser(
    f"~/ros2_ws/src/combined_pkg/benchmark_results/"
    f"{datetime.now().strftime('%Y%m%d_%H%M%S')}"
)


class BenchmarkNode(Node):
    def __init__(self):
        super().__init__("benchmark_controllers")
        self.mode_pub = self.create_publisher(String, "/controller/mode", 10)
        self.target_pub = self.create_publisher(
            Float64MultiArray, "/controller/q_target", 10
        )
        self.pso_pub = self.create_publisher(Bool, "/controller/pso_enable", 10)
        self.gains_pid = self.create_publisher(
            Float64MultiArray, "/controller/gains_pid", 10
        )
        self.gains_smc = self.create_publisher(
            Float64MultiArray, "/controller/gains_smc", 10
        )
        self.gains_mpc = self.create_publisher(
            Float64MultiArray, "/controller/gains_mpc", 10
        )

        self.q = None
        self.lock = threading.Lock()
        self.create_subscription(JointState, "/joint_states", self._js_cb, 50)
        self.pso_gains = {"pid": None, "smc": None, "mpc": None}

    def _js_cb(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        if not all(n in name_to_pos for n in JOINT_NAMES):
            return
        with self.lock:
            self.q = np.array([name_to_pos[n] for n in JOINT_NAMES], dtype=float)

    def get_q(self):
        with self.lock:
            return None if self.q is None else self.q.copy()

    def pub_mode(self, mode: str):
        m = String()
        m.data = mode
        self.mode_pub.publish(m)

    def pub_target(self, q):
        m = Float64MultiArray()
        m.data = [float(x) for x in q]
        for _ in range(5):
            self.target_pub.publish(m)

    def pub_pso(self, on: bool):
        m = Bool()
        m.data = bool(on)
        self.pso_pub.publish(m)

    def pub_gains(self):
        mapping = [
            ("pid", self.gains_pid),
            ("smc", self.gains_smc),
            ("mpc", self.gains_mpc),
        ]
        for key, pub in mapping:
            th = self.pso_gains.get(key)
            if th is None:
                continue
            m = Float64MultiArray()
            m.data = [float(x) for x in th]
            for _ in range(3):
                pub.publish(m)

    def run_pso_once(self):
        self.get_logger().info("Running PSO offline…")
        from pso_optimizer import pso_optimize_all

        results = pso_optimize_all(n_particles=16, n_iters=22, verbose=True)
        for mode, (th, cost) in results.items():
            self.pso_gains[mode] = th
            self.get_logger().info(f"PSO {mode}: cost={cost:.4f}")
        return results

    def go_home(self):
        """
        1) idle + PSO OFF
        2) PID → Q_HOME تا حداکثر HOME_DURATION
        3) وقتی |e|<HOME_TOL، هنوز HOME_HOLD فرمان Home بده
        4) idle + POST_HOME_WAIT قبل از trial بعدی
        """
        self.get_logger().info(">>> GO HOME (up to 10 s, tol=0.05)")
        self.pub_mode("idle")
        time.sleep(0.3)

        self.pub_pso(False)  # همیشه با gain پیش‌فرض
        time.sleep(0.2)

        self.pub_mode("pid")
        dt = 1.0 / RATE_HZ
        t0 = time.time()
        reached_t = None

        while time.time() - t0 < HOME_DURATION:
            self.pub_target(Q_HOME)
            q = self.get_q()
            if q is not None:
                err = float(np.linalg.norm(q - Q_HOME))
                if err < HOME_TOL:
                    if reached_t is None:
                        reached_t = time.time()
                        self.get_logger().info(f"Home entered (|e|={err:.4f})")
                    # حداقل HOME_HOLD ثانیه روی Home بمان
                    if time.time() - reached_t >= HOME_HOLD:
                        break
                else:
                    reached_t = None  # اگر دوباره دور شد، از نو
            time.sleep(dt)

        q = self.get_q()
        err = float(np.linalg.norm(q - Q_HOME)) if q is not None else float("nan")
        if q is not None and err < HOME_TOL:
            self.get_logger().info(
                f"Home OK: {np.round(q, 3)}  |e|={err:.4f}"
            )
        else:
            self.get_logger().warning(
                f"Home NOT fully reached after {HOME_DURATION}s "
                f"(pose={None if q is None else np.round(q, 3)}, |e|={err:.4f}). "
                f"Continuing anyway."
            )

        self.pub_mode("idle")
        self.get_logger().info(
            f"Waiting {POST_HOME_WAIT}s before next controller…"
        )
        time.sleep(POST_HOME_WAIT)

    def run_trial(self, mode: str, pso_on: bool):
        label = f"{mode}_pso{'ON' if pso_on else 'OFF'}"
        self.get_logger().info(f"=== TRIAL {label} ===")

        self.pub_mode("idle")
        time.sleep(0.2)

        self.pub_pso(pso_on)
        if pso_on:
            self.pub_gains()
        time.sleep(0.3)

        self.pub_mode(mode)
        self.pub_target(Q_DES)

        t0 = time.time()
        dt = 1.0 / RATE_HZ
        rows = []
        while time.time() - t0 < DURATION:
            q = self.get_q()
            t = time.time() - t0
            if q is not None:
                e = q - Q_DES
                rows.append(
                    [t]
                    + q.tolist()
                    + e.tolist()
                    + [float(np.linalg.norm(e))]
                )
            self.pub_target(Q_DES)
            if pso_on:
                self.pub_gains()
            time.sleep(dt)

        self.pub_mode("idle")
        time.sleep(SETTLE_AFTER_TRIAL)
        return label, rows


def save_csv(path, rows):
    header = (
        ["t"]
        + [f"q{i+1}" for i in range(N)]
        + [f"e{i+1}" for i in range(N)]
        + ["e_norm"]
    )
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def make_plots(all_data, out_dir, q_des):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 5))
    for label, rows in all_data.items():
        if not rows:
            continue
        arr = np.array(rows)
        ax.plot(arr[:, 0], arr[:, -1], label=label, lw=1.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(r"$\|e\|_2$ (rad)")
    ax.set_title("Tracking error norm — all controllers (from HOME)")
    ax.legend(fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "compare_error_norm.png"), dpi=160)
    plt.close(fig)

    for label, rows in all_data.items():
        if not rows:
            continue
        arr = np.array(rows)
        t = arr[:, 0]

        fig, axes = plt.subplots(3, 2, figsize=(10, 8), sharex=True)
        axes = axes.ravel()
        for i in range(N):
            axes[i].plot(t, arr[:, 1 + i], label="q", lw=1.5)
            axes[i].axhline(q_des[i], color="r", ls="--", label="setpoint")
            axes[i].set_ylabel(JOINT_NAMES[i])
            axes[i].grid(True, alpha=0.3)
            if i == 0:
                axes[i].legend(fontsize=8)
        axes[-1].axis("off")
        axes[4].set_xlabel("Time (s)")
        fig.suptitle(f"{label}")
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, f"traj_{label}.png"), dpi=140)
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(9, 4))
        for i in range(N):
            ax.plot(t, arr[:, 1 + N + i], label=f"e{i+1}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Error (rad)")
        ax.set_title(f"Joint errors — {label}")
        ax.legend(ncol=5, fontsize=8)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, f"err_{label}.png"), dpi=140)
        plt.close(fig)

    labels, final_e, ise = [], [], []
    for label, rows in all_data.items():
        if not rows:
            continue
        arr = np.array(rows)
        labels.append(label)
        final_e.append(float(arr[-1, -1]))
        if len(arr) > 1:
            dtw = np.diff(arr[:, 0])
            ise.append(float(np.sum(arr[1:, -1] ** 2 * dtw)))
        else:
            ise.append(0.0)

    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    x = np.arange(len(labels))
    axes[0].bar(x, final_e, color="steelblue")
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(labels, rotation=30, ha="right", fontsize=8)
    axes[0].set_ylabel(r"final $\|e\|_2$")
    axes[0].set_title("Final error")
    axes[0].grid(True, axis="y", alpha=0.3)

    axes[1].bar(x, ise, color="darkorange")
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(labels, rotation=30, ha="right", fontsize=8)
    axes[1].set_ylabel("ISE")
    axes[1].set_title("Integral Square Error")
    axes[1].grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "summary_bars.png"), dpi=160)
    plt.close(fig)

    with open(os.path.join(out_dir, "summary.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["trial", "final_e_norm", "ISE"])
        for a, b, c in zip(labels, final_e, ise):
            w.writerow([a, b, c])


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    rclpy.init()
    node = BenchmarkNode()

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    node.get_logger().info("Waiting for /joint_states…")
    t0 = time.time()
    while node.get_q() is None and time.time() - t0 < 15.0:
        time.sleep(0.1)
    if node.get_q() is None:
        node.get_logger().error(
            "No /joint_states — start Gazebo + controllers first"
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    node.run_pso_once()

    trials = [
        ("pid", False),
        ("pid", True),
        ("smc", False),
        ("smc", True),
        ("mpc", False),
        ("mpc", True),
    ]

    all_data = {}
    for mode, pso_on in trials:
        if pso_on and node.pso_gains.get(mode) is None:
            node.get_logger().warning(f"Skip {mode} PSO ON — no gains")
            continue

        # ترتیب: Home کامل → مکث ۲s → trial
        node.go_home()
        label, rows = node.run_trial(mode, pso_on)
        all_data[label] = rows
        save_csv(os.path.join(OUT_DIR, f"{label}.csv"), rows)

    node.pub_mode("idle")
    make_plots(all_data, OUT_DIR, Q_DES)

    node.get_logger().info(f"Done. Results in: {OUT_DIR}")
    print("\n=== Results folder ===")
    print(OUT_DIR)
    for name in sorted(os.listdir(OUT_DIR)):
        print(" ", name)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()