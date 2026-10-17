#!/usr/bin/env python3
"""
All-in-one: detector + pick loop + PID + MPC + SMC + control panel GUI.

    ros2 run combined_pkg main.py
"""
import sys
import os
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.executors import MultiThreadedExecutor

from perception import CubeDetector
from motion_control import ArmController
from pid_controller_node import PIDControllerNode
from mpc_controller_node import MPCControllerNode
from smc_controller_node import SMCControllerNode
from control_panel import ControlPanel

from magnet_gripper_node import MagnetGripperNode

def main():
    rclpy.init()

    detector = CubeDetector()
    arm = ArmController(detector)
    pid = PIDControllerNode()
    mpc = MPCControllerNode()
    smc = SMCControllerNode()
    panel = ControlPanel(already_initialized=True)

    threading.Thread(target=arm.run_forever, daemon=True).start()

    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(detector)
    executor.add_node(pid)
    executor.add_node(mpc)
    executor.add_node(smc)

    magnet = MagnetGripperNode()
    executor.add_node(magnet)

    def spin_ros():
        try:
            executor.spin()
        except Exception:
            pass

    threading.Thread(target=spin_ros, daemon=True).start()
    detector.get_logger().info("ALL-IN-ONE: GUI + PID + SMC + MPC + detector")

    try:
        panel.spin_gui()
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        for n in (panel, detector, pid, mpc, smc):
            try:
                n.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()