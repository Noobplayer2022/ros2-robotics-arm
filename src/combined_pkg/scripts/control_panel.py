#!/usr/bin/env python3
"""
Control panel for combined_pkg (ROS2)
- Joint setpoints + Home / PID / SMC / MPC / Clear
- Arm power + color filters
- PSO enable + Run PSO (gains only; controller formulas unchanged)
"""
import os
import sys
import threading
import tkinter as tk
from tkinter import font as tkfont

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float64MultiArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config

C = {
    "bg": "#0f1419",
    "surface": "#1a2332",
    "card": "#243044",
    "border": "#2d3a4f",
    "text": "#e8eef7",
    "muted": "#8b9bb4",
    "accent": "#3b82f6",
    "accent2": "#8b5cf6",
    "success": "#10b981",
    "danger": "#ef4444",
    "warning": "#f59e0b",
    "hover": "#334155",
}
COLORS = ["red", "green", "blue"]
HEX_COLOR = {"red": "#ef4444", "green": "#10b981", "blue": "#3b82f6"}
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
JOINT_LIMITS = [
    (-1.47, 3.142),
    (-1.57, 1.57),
    (-1.571, 3.142),
    (-1.571, 3.142),
    (0.05, 3.142),
]
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.05]


class ControlPanel(Node):
    def __init__(self, already_initialized=False):
        super().__init__("control_panel")

        # از config.py خودت
        self.color_pub = self.create_publisher(
            String, config.ENABLED_COLORS_TOPIC, 1
        )
        self.enable_pub = self.create_publisher(
            Bool, config.ARM_ENABLED_TOPIC, 1
        )

        self.target_pub = self.create_publisher(
            Float64MultiArray, "/controller/q_target", 10
        )
        self.mode_pub = self.create_publisher(String, "/controller/mode", 10)

        # PSO
        self.pso_enable_pub = self.create_publisher(
            Bool, "/controller/pso_enable", 1
        )
        self.gains_pid_pub = self.create_publisher(
            Float64MultiArray, "/controller/gains_pid", 1
        )
        self.gains_smc_pub = self.create_publisher(
            Float64MultiArray, "/controller/gains_smc", 1
        )
        self.gains_mpc_pub = self.create_publisher(
            Float64MultiArray, "/controller/gains_mpc", 1
        )

        self.enabled_color = {c: True for c in COLORS}
        self.arm_on = False
        self._pso_busy = False
        self.best = {"pid": None, "smc": None, "mpc": None}
        self.best_cost = {}

        self.root = tk.Tk()
        self.root.title("Robot Control Center")
        self.root.configure(bg=C["bg"])
        self.root.geometry("940x700")
        self.root.minsize(820, 600)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self._font = tkfont.Font(family="Segoe UI", size=11)
        self._font_b = tkfont.Font(family="Segoe UI", size=12, weight="bold")
        self._font_s = tkfont.Font(family="Segoe UI", size=9)

        self.joint_vars = []
        self.color_btns = {}
        self.sidebar_visible = True

        self._build_ui()
        self.publish_colors()
        self._publish_arm(False)
        self._publish_pso_enable(False)

    # ══════════════════ UI ══════════════════
    def _build_ui(self):
        nav = tk.Frame(self.root, bg=C["surface"], height=52)
        nav.pack(fill="x")
        nav.pack_propagate(False)

        tk.Button(
            nav, text="☰", font=("Segoe UI", 14),
            bg=C["surface"], fg=C["text"], relief="flat",
            command=self._toggle_sidebar, padx=12,
        ).pack(side="left")

        tk.Label(
            nav, text="Robot Control Center",
            bg=C["surface"], fg=C["text"], font=self._font_b,
        ).pack(side="left", padx=8)

        self.lbl_nav = tk.Label(
            nav, text="mode: idle · PSO: OFF",
            bg=C["surface"], fg=C["muted"], font=self._font_s,
        )
        self.lbl_nav.pack(side="right", padx=16)

        body = tk.Frame(self.root, bg=C["bg"])
        body.pack(fill="both", expand=True)

        self.sidebar = tk.Frame(body, bg=C["surface"], width=200)
        self.sidebar.pack(side="left", fill="y")
        self.sidebar.pack_propagate(False)
        self._build_sidebar(self.sidebar)

        self.content = tk.Frame(body, bg=C["bg"])
        self.content.pack(
            side="left", fill="both", expand=True, padx=16, pady=12
        )
        self._build_main(self.content)

    def _build_sidebar(self, parent):
        tk.Label(
            parent, text="ARM POWER", bg=C["surface"],
            fg=C["muted"], font=self._font_s,
        ).pack(anchor="w", padx=16, pady=(16, 4))

        self.btn_arm = tk.Button(
            parent, text="Power: OFF", bg=C["danger"], fg="white",
            relief="flat", font=self._font_b, pady=8,
            command=self._toggle_arm,
        )
        self.btn_arm.pack(fill="x", padx=16, pady=8)

        tk.Label(
            parent, text="COLORS", bg=C["surface"],
            fg=C["muted"], font=self._font_s,
        ).pack(anchor="w", padx=16, pady=(8, 4))

        crow = tk.Frame(parent, bg=C["surface"])
        crow.pack(fill="x", padx=12)
        for color in COLORS:
            b = tk.Button(
                crow, text=color[:1].upper(), width=3,
                bg=HEX_COLOR[color], fg="white", relief="flat",
                command=lambda c=color: self._toggle_color(c),
            )
            b.pack(side="left", padx=4)
            self.color_btns[color] = b

        tk.Button(
            parent, text="All ON", command=self.all_on,
            bg=C["border"], fg=C["text"], relief="flat",
        ).pack(fill="x", padx=16, pady=(8, 2))
        tk.Button(
            parent, text="All OFF", command=self.all_off,
            bg=C["border"], fg=C["text"], relief="flat",
        ).pack(fill="x", padx=16, pady=2)

        self.color_status = tk.Label(
            parent, text="Colors: red, green, blue",
            bg=C["surface"], fg=C["muted"], font=self._font_s, wraplength=170,
        )
        self.color_status.pack(anchor="w", padx=16, pady=8)

        tk.Label(
            parent,
            text="PSO only changes gains.\nController formulas stay the same.",
            bg=C["surface"], fg=C["muted"], font=self._font_s,
            wraplength=170, justify="left",
        ).pack(anchor="w", padx=16, pady=16)

    def _build_main(self, parent):
        card = tk.Frame(parent, bg=C["card"], padx=16, pady=12)
        card.pack(fill="x", pady=(0, 10))

        tk.Label(
            card, text="Joint setpoints (rad)",
            bg=C["card"], fg=C["text"], font=self._font_b,
        ).pack(anchor="w", pady=(0, 8))

        self.joint_vars = []
        for i, name in enumerate(JOINT_NAMES):
            row = tk.Frame(card, bg=C["card"])
            row.pack(fill="x", pady=3)
            tk.Label(
                row, text=name, width=10, anchor="w",
                bg=C["card"], fg=C["muted"], font=self._font,
            ).pack(side="left")
            lo, hi = JOINT_LIMITS[i]
            var = tk.DoubleVar(value=HOME_JOINTS[i])
            self.joint_vars.append(var)
            val_lbl = tk.Label(
                row, text=f"{HOME_JOINTS[i]:.2f}", width=6,
                bg=C["card"], fg=C["text"], font=self._font_s,
            )
            tk.Scale(
                row, from_=lo, to=hi, resolution=0.01,
                orient="horizontal", variable=var,
                bg=C["card"], fg=C["text"],
                troughcolor=C["border"], highlightthickness=0,
                showvalue=0, length=360,
                command=lambda v, lb=val_lbl: lb.config(text=f"{float(v):.2f}"),
            ).pack(side="left", fill="x", expand=True, padx=8)
            val_lbl.pack(side="left")

        btn_row = tk.Frame(card, bg=C["card"])
        btn_row.pack(fill="x", pady=(12, 4))

        def mk(text, color, cmd):
            return tk.Button(
                btn_row, text=text, command=cmd,
                bg=color, fg="white", relief="flat",
                font=self._font_b, padx=14, pady=6,
            )

        mk("Home", C["border"], self._on_home).pack(side="left", padx=4)
        mk("PID", C["accent"], lambda: self._send_setpoint("pid")).pack(
            side="left", padx=4
        )
        mk("SMC", C["warning"], lambda: self._send_setpoint("smc")).pack(
            side="left", padx=4
        )
        mk("MPC", C["accent2"], lambda: self._send_setpoint("mpc")).pack(
            side="left", padx=4
        )
        mk("Clear", C["danger"], self._on_clear).pack(side="left", padx=4)

        self.lbl_cmd = tk.Label(
            card,
            text="Sliders = setpoint · PID/SMC/MPC = controller · PSO = gains only",
            bg=C["card"], fg=C["muted"], font=self._font_s,
        )
        self.lbl_cmd.pack(anchor="w", pady=(8, 0))

        # —— PSO card ——
        pso = tk.Frame(parent, bg=C["card"], padx=16, pady=12)
        pso.pack(fill="x", pady=10)

        tk.Label(
            pso, text="PSO (gain tuning only)",
            bg=C["card"], fg=C["text"], font=self._font_b,
        ).pack(anchor="w")

        tk.Label(
            pso,
            text=(
                "OFF = MATLAB default gains inside each controller.\n"
                "Run PSO = search better gains.  Enable = apply them to PID/SMC/MPC."
            ),
            bg=C["card"], fg=C["muted"], font=self._font_s,
            justify="left", wraplength=680,
        ).pack(anchor="w", pady=(4, 8))

        row = tk.Frame(pso, bg=C["card"])
        row.pack(fill="x")

        self.pso_var = tk.BooleanVar(value=False)
        tk.Checkbutton(
            row, text="Enable PSO gains",
            variable=self.pso_var,
            bg=C["card"], fg=C["text"], selectcolor=C["surface"],
            activebackground=C["card"], activeforeground=C["text"],
            font=self._font, command=self._on_pso_toggle,
        ).pack(side="left")

        tk.Button(
            row, text="Run PSO",
            command=self._run_pso,
            bg=C["accent2"], fg="white", relief="flat",
            font=self._font_b, padx=14, pady=5,
        ).pack(side="right")

        self.pso_status = tk.Label(
            pso, text="PSO: OFF · controllers use default MATLAB gains",
            bg=C["card"], fg=C["muted"], font=self._font_s,
            wraplength=680, justify="left",
        )
        self.pso_status.pack(anchor="w", pady=(8, 0))

    def _toggle_sidebar(self):
        if self.sidebar_visible:
            self.sidebar.pack_forget()
            self.sidebar_visible = False
        else:
            self.sidebar.pack(side="left", fill="y", before=self.content)
            self.sidebar_visible = True

    # ══════════════════ actions ══════════════════
    def _read_joints(self):
        return [float(v.get()) for v in self.joint_vars]

    def _set_sliders(self, values):
        for var, val in zip(self.joint_vars, values):
            var.set(float(val))

    def _nav(self, mode):
        pso = "ON" if self.pso_var.get() else "OFF"
        self.lbl_nav.config(text=f"mode: {mode} · PSO: {pso}")

    def _publish_target(self, q):
        m = Float64MultiArray()
        m.data = [float(x) for x in q]
        for _ in range(3):
            self.target_pub.publish(m)

    def _publish_mode(self, mode: str):
        m = String()
        m.data = mode
        self.mode_pub.publish(m)
        self._nav(mode)

    def _send_setpoint(self, mode: str):
        q = self._read_joints()
        self._publish_mode(mode)
        self._publish_target(q)
        if self.pso_var.get():
            self._publish_all_gains()
        self.lbl_cmd.config(
            text=f"{mode.upper()} {[round(x, 3) for x in q]} · "
                 f"PSO={'ON' if self.pso_var.get() else 'OFF'}"
        )
        self.get_logger().info(f"{mode.upper()} setpoint={q} PSO={self.pso_var.get()}")

    def _on_home(self):
        self._set_sliders(HOME_JOINTS)
        self._publish_mode("pid")
        self._publish_target(HOME_JOINTS)
        if self.pso_var.get():
            self._publish_all_gains()
        self.lbl_cmd.config(text=f"HOME {HOME_JOINTS}")

    def _on_clear(self):
        self._publish_mode("idle")
        self.lbl_cmd.config(text="Cleared · mode=idle")

    def _toggle_arm(self):
        self.arm_on = not self.arm_on
        self._publish_arm(self.arm_on)

    def _publish_arm(self, on: bool):
        m = Bool()
        m.data = on
        self.enable_pub.publish(m)
        self.btn_arm.config(
            text=f"Power: {'ON' if on else 'OFF'}",
            bg=C["success"] if on else C["danger"],
        )
        self.get_logger().info(f"Arm power: {'ON' if on else 'OFF'}")

    def _toggle_color(self, color):
        self.enabled_color[color] = not self.enabled_color[color]
        self.refresh_color_buttons()
        self.publish_colors()

    def all_on(self):
        for c in COLORS:
            self.enabled_color[c] = True
        self.refresh_color_buttons()
        self.publish_colors()

    def all_off(self):
        for c in COLORS:
            self.enabled_color[c] = False
        self.refresh_color_buttons()
        self.publish_colors()

    def refresh_color_buttons(self):
        for color, btn in self.color_btns.items():
            on = self.enabled_color[color]
            btn.config(
                bg=HEX_COLOR[color] if on else C["border"],
                fg="white" if on else C["muted"],
            )

    def publish_colors(self):
        active = [c for c in COLORS if self.enabled_color[c]]
        m = String()
        m.data = ",".join(active)
        self.color_pub.publish(m)
        self.color_status.config(
            text=f"Colors: {', '.join(active) if active else 'none'}"
        )

    # ══════════════════ PSO ══════════════════
    def _publish_pso_enable(self, on: bool):
        m = Bool()
        m.data = bool(on)
        self.pso_enable_pub.publish(m)

    def _publish_all_gains(self):
        mapping = [
            ("pid", self.gains_pid_pub),
            ("smc", self.gains_smc_pub),
            ("mpc", self.gains_mpc_pub),
        ]
        for key, pub in mapping:
            th = self.best.get(key)
            if th is None:
                continue
            msg = Float64MultiArray()
            msg.data = [float(x) for x in th]
            for _ in range(3):
                pub.publish(msg)

    def _on_pso_toggle(self):
        on = bool(self.pso_var.get())
        self._publish_pso_enable(on)
        if on:
            if any(self.best[k] is not None for k in self.best):
                self._publish_all_gains()
                self.pso_status.config(
                    text="PSO: ON · optimized gains sent to controllers"
                )
            else:
                self.pso_status.config(
                    text="PSO: ON · no result yet — press Run PSO"
                )
        else:
            self.pso_status.config(
                text="PSO: OFF · controllers use default MATLAB gains"
            )
        self._nav("—")
        self.get_logger().info(f"PSO enable={on}")

    def _run_pso(self):
        if self._pso_busy:
            return
        self._pso_busy = True
        self.pso_status.config(text="PSO: running (PID + SMC + MPC)…")

        def job():
            try:
                from pso_optimizer import pso_optimize_all
                results = pso_optimize_all(
                    n_particles=16, n_iters=22, verbose=False
                )
                for mode, (th, cost) in results.items():
                    self.best[mode] = th
                    self.best_cost[mode] = cost

                def ui_ok():
                    self.pso_var.set(True)
                    self._publish_pso_enable(True)
                    self._publish_all_gains()
                    msg = (
                        f"PSO: done · "
                        f"PID={self.best_cost.get('pid', 0):.3f} · "
                        f"SMC={self.best_cost.get('smc', 0):.3f} · "
                        f"MPC={self.best_cost.get('mpc', 0):.3f} · gains APPLIED"
                    )
                    self.pso_status.config(text=msg)
                    self._nav("—")
                    self._pso_busy = False
                    self.get_logger().info(msg)

                self.root.after(0, ui_ok)
            except Exception as e:
                def ui_err():
                    self.pso_status.config(text=f"PSO error: {e}")
                    self._pso_busy = False
                    self.get_logger().error(str(e))

                self.root.after(0, ui_err)

        threading.Thread(target=job, daemon=True).start()

    # ══════════════════ lifecycle ══════════════════
    def on_close(self):
        try:
            self.root.quit()
            self.root.destroy()
        except Exception:
            pass

    def spin_gui(self):
        def pump():
            try:
                rclpy.spin_once(self, timeout_sec=0.03)
            except Exception:
                pass
            try:
                if self.root.winfo_exists():
                    self.root.after(40, pump)
            except Exception:
                pass

        self.root.after(40, pump)
        self.root.mainloop()


def main():
    rclpy.init()
    panel = ControlPanel(already_initialized=True)
    try:
        panel.spin_gui()
    finally:
        panel.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()