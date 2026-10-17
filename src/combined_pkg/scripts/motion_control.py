#!/usr/bin/env python3
"""
Safe side pick-and-place:
  approach → pick → lift → PLACE → release → HOME
Magnet stays ON until the cube is at the place pose.
"""
import time
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool
from builtin_interfaces.msg import Duration
import config

# ---------- FK / IK ----------
JOINT_DEF = [
    ("z", np.array([0.0, 0.0, 0.12]), np.array([0.0, 0.0, 0.0])),
    ("y", np.array([0.0, 0.0, 0.25]), np.array([0.0, 0.0, 0.0])),
    ("y", np.array([0.0, 0.0, 0.25]), np.array([0.0, 0.0, 0.0])),
    ("y", np.array([0.0, 0.0, 0.18]), np.array([0.0, 0.0, 0.0])),
    ("z", np.array([0.0, 0.0, 0.10]), np.array([0.0, 0.0, 0.0])),
]

Q_MIN = np.array([-1.47, -1.57, -1.571, -1.571, 0.05])
Q_MAX = np.array([3.142, 1.57, 3.142, 3.142, 3.142])

GRIPPER_OPEN = list(getattr(config, "GRIPPER_OPEN_JOINTS", [0.03, -0.03]))
GRIPPER_CLOSED = list(getattr(config, "GRIPPER_CLOSED_JOINTS", [0.0, 0.0]))
GRIPPER_NAMES = list(getattr(config, "GRIPPER_JOINT_NAMES", ["joint_6", "joint_7"]))

SAFE_Z = 0.32
PICK_Z = 0.14
LIFT_Z = 0.35
APPROACH_SCALE = 0.45


def _rotx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _roty(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rotz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _T(axis, q, origin):
    if axis == "z":
        R = _rotz(q)
    elif axis == "y":
        R = _roty(q)
    else:
        R = _rotx(q)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = origin
    return T


def fk(q):
    T = np.eye(4)
    for i, (axis, origin, _) in enumerate(JOINT_DEF):
        T = T @ _T(axis, float(q[i]), origin)
    return T[:3, 3].copy()


def ik(target, q0=None, iters=80, step=0.28):
    q = np.array(
        q0 if q0 is not None else [0.0, 0.3, -0.4, 0.0, 1.0],
        dtype=float,
    )
    q = np.clip(q, Q_MIN, Q_MAX)
    target = np.asarray(target, dtype=float)
    for _ in range(iters):
        p = fk(q)
        e = target - p
        if np.linalg.norm(e) < 0.008:
            break
        J = np.zeros((3, 5))
        eps = 1e-4
        for i in range(5):
            dq = np.zeros(5)
            dq[i] = eps
            J[:, i] = (fk(np.clip(q + dq, Q_MIN, Q_MAX)) - p) / eps
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(3), e)
        q = np.clip(q + step * dq, Q_MIN, Q_MAX)
    err = float(np.linalg.norm(fk(q) - target))
    return q, err


class ArmController:
    def __init__(self, detector):
        # detector = CubeDetector (Node + predict_setpoint)
        self.node = detector
        self.detector = detector
        self.log = detector.get_logger()
        self.enabled = False
        self.busy = False
        self.q = np.zeros(5)
        self.q_grip = np.array(GRIPPER_OPEN, dtype=float)

        self.arm_pub = detector.create_publisher(
            JointTrajectory, config.CONTROLLER_TOPIC, 10
        )
        self.grip_pub = detector.create_publisher(
            JointTrajectory, "/gripper_controller/joint_trajectory", 10
        )
        self.magnet_pub = detector.create_publisher(Bool, "/magnet/attach", 10)

        detector.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        detector.create_subscription(
            Bool, config.ARM_ENABLED_TOPIC, self._en_cb, 10
        )

        self.log.info(
            "ArmController ready (pick → lift → PLACE → release → HOME)"
        )

    def _js_cb(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self.q = np.array(
                [name_to_pos[n] for n in config.JOINT_NAMES], dtype=float
            )
        except KeyError:
            pass
        try:
            self.q_grip = np.array(
                [name_to_pos[n] for n in GRIPPER_NAMES], dtype=float
            )
        except KeyError:
            pass

    def _en_cb(self, msg: Bool):
        self.enabled = bool(msg.data)
        self.log.info("Arm ENABLED" if self.enabled else "Arm DISABLED")

    def set_magnet(self, on: bool):
        m = Bool()
        m.data = bool(on)
        for _ in range(3):
            self.magnet_pub.publish(m)
            time.sleep(0.05)
        self.log.info("Magnet ON" if on else "Magnet OFF")

    def go_joints(self, positions, duration=2.0):
        positions = list(
            np.clip(np.asarray(positions, dtype=float), Q_MIN, Q_MAX)
        )
        traj = JointTrajectory()
        traj.joint_names = list(config.JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = positions
        sec = int(duration)
        nsec = int((duration - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)
        traj.points = [pt]
        self.arm_pub.publish(traj)
        time.sleep(duration + 0.35)

    def go_gripper(self, positions, duration=0.9):
        traj = JointTrajectory()
        traj.joint_names = list(GRIPPER_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        sec = int(duration)
        nsec = int((duration - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)
        traj.points = [pt]
        self.grip_pub.publish(traj)
        # fallback اگر gripper_controller بالا نباشد
        try:
            self.arm_pub.publish(traj)
        except Exception:
            pass
        time.sleep(duration + 0.25)

    def go_xyz(self, xyz, duration=2.2, seed=None):
        q0 = seed if seed is not None else self.q.copy()
        q, err = ik(xyz, q0=q0)
        self.log.info(
            f"IK → ({xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f})  err={err:.4f} m"
        )
        self.go_joints(q, duration)
        return q

    def pick_and_place(self, x, y, z):
        """x,y,z در frame base_link"""
        self.log.info(
            f"=== PICK-PLACE  cube@base ({x:.3f},{y:.3f},{z:.3f}) ==="
        )

        approach = [x * APPROACH_SCALE, y * APPROACH_SCALE, SAFE_Z]
        above = [x, y, SAFE_Z]
        pick = [x, y, max(PICK_Z, z + 0.04)]
        lift = [x, y, LIFT_Z]

        place_joints = list(
            getattr(
                config, "PLACE_JOINTS", [-1.381, 0.798, 1.060, 0.0, 0.5]
            )
        )
        home_joints = list(
            getattr(config, "HOME_JOINTS", [0.0, 0.0, 0.0, 0.0, 0.05])
        )
        ready_joints = list(
            getattr(
                config, "READY_JOINTS", [-0.3863, 1.244, -1.262, 0.0, 1.6311]
            )
        )

        # 0) آماده‌سازی
        self.set_magnet(False)
        self.go_gripper(GRIPPER_OPEN, 0.8)
        self.go_joints(ready_joints, 2.0)
        self.log.info("Ready pose")

        # 1) نزدیک شدن از بغل
        q1 = self.go_xyz(approach, 2.4, seed=ready_joints)
        self.log.info("Side approach")

        # 2) بالای مکعب
        q2 = self.go_xyz(above, 2.2, seed=q1)
        self.log.info("Above cube")

        # 3) پایین برای برداشتن
        q3 = self.go_xyz(pick, 2.0, seed=q2)
        self.log.info("At pick height")
        time.sleep(0.4)

        # 4) چسباندن
        self.set_magnet(True)
        time.sleep(0.5)
        self.go_gripper(GRIPPER_CLOSED, 0.8)
        self.log.info("Gripper CLOSED + magnet ON")
        time.sleep(0.4)

        # 5) بالا آوردن (مگنت هنوز روشن)
        q4 = self.go_xyz(lift, 2.2, seed=q3)
        self.log.info("Lifted (cube still held)")

        # 6) بردن به محل گذاشتن
        self.go_joints(place_joints, 2.8)
        self.log.info("At PLACE pose")
        time.sleep(0.5)

        # 7) رها کردن
        self.set_magnet(False)
        time.sleep(0.3)
        self.go_gripper(GRIPPER_OPEN, 0.8)
        self.log.info("Cube RELEASED")
        time.sleep(0.5)

        # 8) کمی عقب
        try:
            place_xyz = fk(np.array(place_joints))
            retreat = [
                place_xyz[0],
                place_xyz[1],
                min(place_xyz[2] + 0.12, LIFT_Z),
            ]
            self.go_xyz(retreat, 1.6, seed=place_joints)
        except Exception:
            pass

        # 9) خانه
        self.go_joints(home_joints, 2.5)
        self.log.info("=== PICK-PLACE done (home) ===")

    def run_forever(self):
        self.log.info(
            "Loop: Arm ON → one full pick-place (place + release + home)"
        )
        period = 0.25
        while True:
            try:
                import rclpy

                if not rclpy.ok():
                    break
            except Exception:
                break
            if (not self.enabled) or self.busy:
                time.sleep(period)
                continue
            target = self.detector.predict_setpoint()
            if target is None:
                time.sleep(period)
                continue
            self.busy = True
            try:
                self.pick_and_place(*target)
            except Exception as e:
                self.log.error(f"pick-place failed: {e}")
                try:
                    self.set_magnet(False)
                    self.go_gripper(GRIPPER_OPEN, 0.5)
                except Exception:
                    pass
            finally:
                self.busy = False
                self.enabled = False
            time.sleep(0.6)