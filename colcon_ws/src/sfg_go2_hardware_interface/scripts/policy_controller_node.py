#!/usr/bin/env python3

import os
import threading
from typing import Dict, Optional

import numpy as np
import rclpy
import torch
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray, String

# ============================================================
# Policy / Control Constants
# ============================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_POLICY_PATH = os.path.join(SCRIPT_DIR, "policy.pt")

CONTROL_RATE_HZ = 50.0
ACTION_SCALE = 0.25

# This is the joint order used by the Isaac/RSL-RL policy.
# The observation q_rel, dq, previous_action, and neural-network output
# should stay in this order.
POLICY_JOINT_ORDER = [
    "front_left_hip_joint",
    "front_left_thigh_joint",
    "front_left_calf_joint",
    "front_right_hip_joint",
    "front_right_thigh_joint",
    "front_right_calf_joint",
    "rear_left_hip_joint",
    "rear_left_thigh_joint",
    "rear_left_calf_joint",
    "rear_right_hip_joint",
    "rear_right_thigh_joint",
    "rear_right_calf_joint",
]

# This is the physical motor / LowState order seen from your Go2 joint_states.
# /go2/policy_lowcmd is published in this order so the C++ low-level controller
# can safely map msg.data[i] to motor_cmd[i].
UNITREE_MOTOR_ORDER = [
    "front_right_hip_joint",
    "front_right_thigh_joint",
    "front_right_calf_joint",
    "front_left_hip_joint",
    "front_left_thigh_joint",
    "front_left_calf_joint",
    "rear_right_hip_joint",
    "rear_right_thigh_joint",
    "rear_right_calf_joint",
    "rear_left_hip_joint",
    "rear_left_thigh_joint",
    "rear_left_calf_joint",
]

# Default standing pose in POLICY_JOINT_ORDER.
# This matches the common Isaac Go2 policy convention:
# FL, FR, RL, RR with hip signs [+,-,+,-].
DEFAULT_JOINT_POS_POLICY = np.array(
    [
        0.1,
        0.8,
        -1.5,
        -0.1,
        0.8,
        -1.5,
        0.1,
        1.0,
        -1.5,
        -0.1,
        1.0,
        -1.5,
    ],
    dtype=np.float32,
)

# Safety limits for manual command input.
MAX_VX = 0.20  # m/s
MAX_VY = 0.10  # m/s
MAX_YAW_RATE = 0.30  # rad/s


def reorder_vector(values: np.ndarray, source_order, target_order) -> np.ndarray:
    """
    Reorder a 12-element joint vector from source_order to target_order.
    """
    if values.shape != (12,):
        raise ValueError(f"Expected 12 values, got shape {values.shape}")

    value_by_name = {name: float(values[i]) for i, name in enumerate(source_order)}
    return np.array([value_by_name[name] for name in target_order], dtype=np.float32)


class PolicyController(Node):
    def __init__(self):
        super().__init__("policy_controller")

        # ----------------------------------------------------
        # Load Policy
        # ----------------------------------------------------
        self.policy_path = (
            self.declare_parameter("policy_path", DEFAULT_POLICY_PATH)
            .get_parameter_value()
            .string_value
        )

        if not os.path.exists(self.policy_path):
            raise FileNotFoundError(f"Policy file not found: {self.policy_path}")

        self.get_logger().info(f"Loading policy from: {self.policy_path}")

        # This assumes policy.pt is a TorchScript policy, which matches the
        # way your current node has been running successfully.
        self.policy = torch.jit.load(self.policy_path, map_location="cpu")
        self.policy.eval()

        self.get_logger().info("Policy loaded successfully.")

        # ----------------------------------------------------
        # State Storage
        # ----------------------------------------------------
        self.state_lock = threading.Lock()
        self.command_lock = threading.Lock()

        self.latest_joint_pos: Dict[str, float] = {}
        self.latest_joint_vel: Dict[str, float] = {}

        self.latest_base_lin_vel = np.zeros(3, dtype=np.float32)
        self.latest_base_ang_vel = np.zeros(3, dtype=np.float32)
        self.latest_projected_gravity = np.array([0.0, 0.0, -1.0], dtype=np.float32)

        # Manual command used inside the policy observation:
        # [vx, vy, yaw_rate]
        self.commands = np.zeros(3, dtype=np.float32)

        # Previous raw neural-network action in POLICY_JOINT_ORDER.
        self.previous_action = np.zeros(12, dtype=np.float32)

        self.have_joint_state = False
        self.have_imu = False
        self.have_base_lin_vel = False

        # ----------------------------------------------------
        # Subscribers
        # ----------------------------------------------------
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/local/sfg_go2_01/joint_states",
            self.joint_state_callback,
            10,
        )

        self.imu_sub = self.create_subscription(
            Imu,
            "/go2/imu",
            self.imu_callback,
            10,
        )

        self.base_lin_vel_sub = self.create_subscription(
            Vector3Stamped,
            "/go2/base_lin_vel",
            self.base_lin_vel_callback,
            10,
        )

        self.policy_cmd_sub = self.create_subscription(
            Float32MultiArray,
            "/go2/policy_cmd",
            self.policy_cmd_callback,
            10,
        )

        # ----------------------------------------------------
        # Publishers
        # ----------------------------------------------------
        self.lowcmd_pub = self.create_publisher(
            Float32MultiArray,
            "/go2/policy_lowcmd",
            10,
        )

        self.policy_obs_pub = self.create_publisher(
            Float32MultiArray,
            "/go2/policy_obs",
            10,
        )

        self.policy_status_pub = self.create_publisher(
            String,
            "/go2/policy_status",
            10,
        )

        self.policy_joint_pub = self.create_publisher(
            JointState,
            "/policy_joint_states",
            10,
        )

        # ----------------------------------------------------
        # Timers
        # ----------------------------------------------------
        self.control_timer = self.create_timer(
            1.0 / CONTROL_RATE_HZ,
            self.control_loop,
        )

        self.status_timer = self.create_timer(
            1.0,
            self.publish_status,
        )

        self.get_logger().info(
            "Manual policy command topic enabled: /go2/policy_cmd = [vx, vy, yaw_rate]"
        )
        self.get_logger().info("Policy controller started.")

    # ========================================================
    # ROS Callbacks
    # ========================================================

    def joint_state_callback(self, msg: JointState):
        with self.state_lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self.latest_joint_pos[name] = float(msg.position[i])
                if i < len(msg.velocity):
                    self.latest_joint_vel[name] = float(msg.velocity[i])

            self.have_joint_state = True

    def imu_callback(self, msg: Imu):
        quat = np.array(
            [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            ],
            dtype=np.float32,
        )

        ang_vel = np.array(
            [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ],
            dtype=np.float32,
        )

        if not np.all(np.isfinite(quat)) or not np.all(np.isfinite(ang_vel)):
            self.get_logger().warn(
                "Received non-finite IMU data. Ignoring IMU message."
            )
            return

        # If quaternion is invalid, skip this IMU update.
        quat_norm = np.linalg.norm(quat)
        if quat_norm < 1e-6:
            self.get_logger().warn(
                "Received near-zero IMU quaternion. Ignoring IMU message."
            )
            return

        quat = quat / quat_norm

        try:
            rot = R.from_quat(quat)

            # Gravity direction expressed in the robot/body frame.
            # The policy expects projected gravity as a 3D observation term.
            projected_gravity = rot.inv().apply(
                np.array([0.0, 0.0, -1.0], dtype=np.float32)
            )
            projected_gravity = projected_gravity.astype(np.float32)

        except Exception as exc:
            self.get_logger().warn(f"Failed to compute projected gravity: {exc}")
            return

        with self.state_lock:
            self.latest_base_ang_vel = ang_vel
            self.latest_projected_gravity = projected_gravity
            self.have_imu = True

    def base_lin_vel_callback(self, msg: Vector3Stamped):
        lin_vel = np.array(
            [
                msg.vector.x,
                msg.vector.y,
                msg.vector.z,
            ],
            dtype=np.float32,
        )

        if not np.all(np.isfinite(lin_vel)):
            self.get_logger().warn(
                f"Received non-finite base linear velocity: {lin_vel}. Ignoring message."
            )
            return

        with self.state_lock:
            self.latest_base_lin_vel = lin_vel
            self.have_base_lin_vel = True

    def policy_cmd_callback(self, msg: Float32MultiArray):
        """
        Receives manual velocity commands for the policy observation.

        Expected:
            /go2/policy_cmd std_msgs/msg/Float32MultiArray
            data = [vx, vy, yaw_rate]

        This does NOT directly command the robot.
        It only changes entries 9-11 of the policy observation vector.
        """

        if len(msg.data) != 3:
            self.get_logger().warn(
                f"Received /go2/policy_cmd with {len(msg.data)} elements. "
                "Expected 3: [vx, vy, yaw_rate]."
            )
            return

        cmd = np.array(msg.data, dtype=np.float32)

        if not np.all(np.isfinite(cmd)):
            self.get_logger().warn(
                f"Received non-finite /go2/policy_cmd: {cmd}. Ignoring command."
            )
            return

        vx = float(np.clip(cmd[0], -MAX_VX, MAX_VX))
        vy = float(np.clip(cmd[1], -MAX_VY, MAX_VY))
        yaw_rate = float(np.clip(cmd[2], -MAX_YAW_RATE, MAX_YAW_RATE))

        safe_cmd = np.array([vx, vy, yaw_rate], dtype=np.float32)

        with self.command_lock:
            self.commands = safe_cmd

        self.get_logger().info(
            f"Updated manual policy command: vx={vx:.3f}, vy={vy:.3f}, yaw_rate={yaw_rate:.3f}"
        )

    # ========================================================
    # Observation Construction
    # ========================================================

    def build_observation(self) -> Optional[np.ndarray]:
        with self.state_lock:
            if not self.have_joint_state:
                self.get_logger().warn("Waiting for /local/sfg_go2_01/joint_states...")
                return None

            if not self.have_imu:
                self.get_logger().warn("Waiting for /go2/imu...")
                return None

            if not self.have_base_lin_vel:
                self.get_logger().warn("Waiting for /go2/base_lin_vel...")
                return None

            missing = [
                name
                for name in POLICY_JOINT_ORDER
                if name not in self.latest_joint_pos
                or name not in self.latest_joint_vel
            ]

            if missing:
                self.get_logger().warn(f"Waiting for joint states for: {missing}")
                return None

            q = np.array(
                [self.latest_joint_pos[name] for name in POLICY_JOINT_ORDER],
                dtype=np.float32,
            )

            dq = np.array(
                [self.latest_joint_vel[name] for name in POLICY_JOINT_ORDER],
                dtype=np.float32,
            )

            base_lin_vel = self.latest_base_lin_vel.copy()
            base_ang_vel = self.latest_base_ang_vel.copy()
            projected_gravity = self.latest_projected_gravity.copy()

        with self.command_lock:
            commands = self.commands.copy()

        q_rel = q - DEFAULT_JOINT_POS_POLICY

        obs = np.concatenate(
            [
                base_lin_vel,  # 3
                base_ang_vel,  # 3
                projected_gravity,  # 3
                commands,  # 3
                q_rel,  # 12
                dq,  # 12
                self.previous_action,  # 12
            ]
        ).astype(np.float32)

        if obs.shape != (48,):
            self.get_logger().error(
                f"Observation has shape {obs.shape}; expected (48,)."
            )
            return None

        if not np.all(np.isfinite(obs)):
            self.get_logger().error(f"Observation contains NaN/Inf. obs={obs}")
            return None

        return obs

    # ========================================================
    # Main Control Loop
    # ========================================================

    def control_loop(self):
        obs = self.build_observation()

        if obs is None:
            return

        # ----------------------------------------------------
        # Publish Observation Debug
        # ----------------------------------------------------
        obs_msg = Float32MultiArray()
        obs_msg.data = [float(x) for x in obs]
        self.policy_obs_pub.publish(obs_msg)

        # ----------------------------------------------------
        # Policy Inference
        # ----------------------------------------------------
        obs_tensor = torch.from_numpy(obs).unsqueeze(0)

        with torch.no_grad():
            action_tensor = self.policy(obs_tensor)

        action = action_tensor.squeeze(0).cpu().numpy().astype(np.float32)

        if action.shape != (12,):
            self.get_logger().error(
                f"Policy action has shape {action.shape}; expected (12,)."
            )
            return

        if not np.all(np.isfinite(action)):
            self.get_logger().error(f"Policy action contains NaN/Inf. action={action}")
            return

        # ----------------------------------------------------
        # Convert Action -> Joint Targets
        # ----------------------------------------------------
        # target_q_policy stays in POLICY_JOINT_ORDER.
        target_q_policy = DEFAULT_JOINT_POS_POLICY + ACTION_SCALE * action

        if not np.all(np.isfinite(target_q_policy)):
            self.get_logger().error(
                f"target_q_policy contains NaN/Inf: {target_q_policy}"
            )
            return

        # Store previous raw action for the next observation.
        self.previous_action = action.copy()

        # ----------------------------------------------------
        # Publish Low Command
        # ----------------------------------------------------
        # The C++ low-level controller maps msg.data[i] to Unitree motor i.
        # Therefore publish this in UNITREE_MOTOR_ORDER, not policy order.
        target_q_unitree = reorder_vector(
            target_q_policy,
            source_order=POLICY_JOINT_ORDER,
            target_order=UNITREE_MOTOR_ORDER,
        )

        lowcmd_msg = Float32MultiArray()
        lowcmd_msg.data = [float(x) for x in target_q_unitree]
        self.lowcmd_pub.publish(lowcmd_msg)

        # ----------------------------------------------------
        # Publish Policy Joint States for Visualization/Debug
        # ----------------------------------------------------
        policy_js = JointState()
        policy_js.header.stamp = self.get_clock().now().to_msg()
        policy_js.header.frame_id = "base"
        policy_js.name = list(POLICY_JOINT_ORDER)
        policy_js.position = [float(x) for x in target_q_policy]
        policy_js.velocity = [0.0 for _ in range(12)]
        policy_js.effort = [0.0 for _ in range(12)]
        self.policy_joint_pub.publish(policy_js)

    def publish_status(self):
        msg = String()
        msg.data = "running"
        self.policy_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = PolicyController()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
