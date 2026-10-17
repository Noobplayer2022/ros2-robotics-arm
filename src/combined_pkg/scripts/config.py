#!/usr/bin/env python3
"""Shared constants — ROS2 compatible."""

MIN_RADIUS = 0.15
MAX_RADIUS = 1.05
CONVEYOR_SURFACE_Z = 0.09
BELT_SAFETY_CLEARANCE = 0.02
MIN_Z = CONVEYOR_SURFACE_Z
MAX_Z = 0.6

HOVER_OFFSET = 0.12
PICK_OFFSET = 0.035
CUBE_SIZE_M = 0.05
PLACE_XYZ = (0.0, 0.20, 0.15)

READY_JOINTS = [-0.3863, 1.244, -1.262, 0.0, 1.6311]
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0]
PLACE_JOINTS = [-1.381, 0.798, 1.060, 0.0, 0.0]
JOINT1_MIN = -1.47
JOINT1_MAX = 3.142

GRIPPER_JOINT_NAMES = ["joint_6", "joint_7"]
GRIPPER_OPEN_JOINTS = [0.03, -0.03]
GRIPPER_CLOSED_JOINTS = [0.0, 0.0]

# ROS2 joint_trajectory_controller topic
CONTROLLER_TOPIC = "/arm_controller/joint_trajectory"
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
BASE_LINK = "base_link"
TIP_LINK = "link_5"

MAGNET_ATTACH_TOPIC = "/magnet/attach"
MAGNET_STATUS_TOPIC = "/magnet/status"

ENABLED_COLORS_TOPIC = "/enabled_colors"
ARM_ENABLED_TOPIC = "/arm_enabled"

CONTROL_RATE = 15.0
STREAM_DT = 1.0 / CONTROL_RATE
TRAJ_POINT_HORIZON = 0.15
TRACK_LEAD_TIME = 0.20

SAFE_HOVER_Z = CONVEYOR_SURFACE_Z + 0.06
JOINT_STEP_LIMIT = 0.12
TRACK_KP = 3.0
MAX_JOINT_VEL = 1.2

HISTORY_MAXLEN = 12
MIN_HISTORY_SPAN = 0.3
MIN_HISTORY_POINTS = 6
MAX_BELT_SPEED = 0.35
MAX_TRACK_AGE = 1.0

# Camera topics — با setup فعلی تو (RGB). اگر depth اضافه کردی عوض کن.
CAMERA_RGB_TOPIC = "/camera/image"
CAMERA_DEPTH_TOPIC = "/camera/depth/image_raw"
CAMERA_POINTS_TOPIC = "/camera/depth/points"
CAMERA_FRAME = "camera"


# موقعیت مکعب در world (همان SPAWN در cube_spawner)
CUBE_WORLD_XYZ = (-0.35, -0.70, 0.10)
CUBE_POSE_TOPIC = "/cube/pose"
USE_FIXED_CUBE_POSE = True

HOVER_OFFSET = 0.12
PICK_OFFSET = 0.04

PLACE_JOINTS = [-1.381, 0.798, 1.060, 0.0, 0.5]
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.05]
READY_JOINTS = [-0.3863, 1.244, -1.262, 0.0, 1.6311]
GRIPPER_JOINT_NAMES = ["joint_6", "joint_7"]
GRIPPER_OPEN_JOINTS = [0.03, -0.03]
GRIPPER_CLOSED_JOINTS = [0.0, 0.0]

CONTROLLER_TOPIC = "/arm_controller/joint_trajectory"
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
BASE_LINK = "base_link"
ARM_ENABLED_TOPIC = "/arm_enabled"
ENABLED_COLORS_TOPIC = "/enabled_colors"