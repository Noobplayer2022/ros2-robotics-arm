#!/usr/bin/env python3
"""
ROS2 shared base for PID / SMC / Fuzzy-SMC / MPC.

Publishes:
    /arm_controller/joint_trajectory
Subscribes:
    /joint_states
    /controller/q_target   (Float64MultiArray, 5 rad)
    /controller/mode       (String: "pid" | "smc" | "mpc" | "idle")
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
N = len(JOINT_NAMES)
CONTROLLER_TOPIC = "/arm_controller/joint_trajectory"
TARGET_TOPIC = "/controller/q_target"
TARGET_TOPIC_LATCHED = "/controller/q_target_latched"
MODE_TOPIC = "/controller/mode"

JOINT_LIMITS = {
    "joint_1": (-1.47, 3.142),
    "joint_2": (-1.57, 1.57),
    "joint_3": (-1.571, 3.142),
    "joint_4": (-1.571, 3.142),
    "joint_5": (0.05, 3.142),
}
Q_MIN = np.array([JOINT_LIMITS[j][0] for j in JOINT_NAMES])
Q_MAX = np.array([JOINT_LIMITS[j][1] for j in JOINT_NAMES])

DT = 0.02
DQ_MAX = 2.0
DDQ_MAX = 20.0


class ControllerNodeBase(Node):
    """
    Subclass and implement:
        compute_ddq(self, e, de, q_des, dq_des, ddq_des) -> np.ndarray (5,)

    mode_name: "pid" | "smc" | "mpc"
    Only streams while /controller/mode == mode_name.
    """

    def __init__(self, node_name: str, mode_name: str):
        super().__init__(node_name)
        self.mode_name = mode_name.strip().lower()
        self.active = False

        self.q = None
        self.dq = np.zeros(N)
        self._last_q = None
        self._last_t = None

        self.q_target = None
        self.q_cmd = None
        self.dq_cmd = np.zeros(N)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, qos)
        self.create_subscription(Float64MultiArray, TARGET_TOPIC, self._target_cb, 10)
        self.create_subscription(String, MODE_TOPIC, self._mode_cb, 10)

        self.target_echo_pub = self.create_publisher(
            Float64MultiArray, TARGET_TOPIC_LATCHED, 10
        )
        self.traj_pub = self.create_publisher(JointTrajectory, CONTROLLER_TOPIC, 10)
        self.create_timer(DT, self._control_loop)

        self.get_logger().info(
            f"{node_name} ready. Inactive until {MODE_TOPIC} == '{self.mode_name}'. "
            f"Setpoint: {TARGET_TOPIC}"
        )

    def _mode_cb(self, msg: String):
        want = msg.data.strip().lower()
        was = self.active
        self.active = want == self.mode_name
        if self.active and not was:
            self.get_logger().info(f"{self.mode_name.upper()} ACTIVE")
        elif not self.active and was:
            self.get_logger().info(f"{self.mode_name.upper()} inactive")

    def _joint_state_cb(self, msg: JointState):
        try:
            q = np.array([msg.position[msg.name.index(j)] for j in JOINT_NAMES])
        except ValueError:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_q is not None and self._last_t is not None:
            dt = now - self._last_t
            if dt > 1e-4:
                self.dq = (q - self._last_q) / dt
        self.q = q
        self._last_q = q
        self._last_t = now
        if self.q_cmd is None:
            self.q_cmd = q.copy()

    def _target_cb(self, msg: Float64MultiArray):
        if len(msg.data) != N:
            self.get_logger().warn(
                f"{TARGET_TOPIC} expected {N} values, got {len(msg.data)}"
            )
            return
        self.q_target = np.clip(np.array(msg.data, dtype=float), Q_MIN, Q_MAX)
        self.get_logger().info(f"New target: {self.q_target}")
        echo = Float64MultiArray()
        echo.data = self.q_target.tolist()
        self.target_echo_pub.publish(echo)

    def compute_ddq(self, e, de, q_des, dq_des, ddq_des):
        raise NotImplementedError

    def stream(self, q_cmd: np.ndarray):
        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = q_cmd.tolist()
        pt.time_from_start = Duration(sec=0, nanosec=int(DT * 2 * 1e9))
        traj.points = [pt]
        self.traj_pub.publish(traj)

    def _control_loop(self):
        # فقط وقتی این کنترلر انتخاب شده باشد
        if not self.active:
            return
        if self.q is None or self.q_target is None or self.q_cmd is None:
            return

        q_des = self.q_target
        dq_des = np.zeros(N)
        ddq_des = np.zeros(N)

        e = self.q_cmd - q_des
        de = self.dq_cmd - dq_des

        ddq_cmd = self.compute_ddq(e, de, q_des, dq_des, ddq_des)
        ddq_cmd = np.clip(ddq_cmd, -DDQ_MAX, DDQ_MAX)

        self.dq_cmd = np.clip(self.dq_cmd + ddq_cmd * DT, -DQ_MAX, DQ_MAX)
        self.q_cmd = np.clip(self.q_cmd + self.dq_cmd * DT, Q_MIN, Q_MAX)
        self.stream(self.q_cmd)

    def run(self):
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        finally:
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()