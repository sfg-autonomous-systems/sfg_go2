#!/usr/bin/env python3

import math
from typing import Optional

import numpy as np
import rclpy
import torch
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray, String

# ============================================================
# Isaac Lab Go2 Constants
# ============================================================

ACTION_SCALE = 0.25

DEFAULT_JOINT_POS = np.array(
    [
        0.1,
        0.8,
        -1.5,  # FL
        -0.1,
        0.8,
        -1.5,  # FR
        0.1,
        1.0,
        -1.5,  # RL
        -0.1,
        1.0,
        -1.5,  # RR
    ],
    dtype=np.float32,
)


# ============================================================
# Policy Joint Order
# (Isaac Lab order)
# ============================================================

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


# ============================================================
# Live Robot Joint Order
# ============================================================

LIVE_JOINT_ORDER = [
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


# ============================================================
# Node
# ============================================================


class PolicyControllerNode(Node):
    def __init__(self):

        super().__init__("policy_controller")

        self.get_logger().info("Starting Policy Controller")

        # ----------------------------------------------------
        # Load Policy
        # ----------------------------------------------------

        self.policy = torch.jit.load("policy.pt")

        self.policy.eval()

        self.get_logger().info("Loaded TorchScript policy")

        # ----------------------------------------------------
        # ROS Topics
        # ----------------------------------------------------

        self.cmd_vel_topic = "/global/sfg_go2_01/locomotion_controller/cmd_vel"

        self.joint_state_topic = "/local/sfg_go2_01/joint_states"

        self.imu_topic = "/go2/imu"

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

        self.get_logger().info("CREATED POLICY JOINT PUBLISHER")

        # ----------------------------------------------------
        # Publish Test JointState
        # ----------------------------------------------------

        test_msg = JointState()

        test_msg.header.frame_id = "base"

        test_msg.name = POLICY_JOINT_ORDER

        test_msg.position = [0.0] * 12

        test_msg.velocity = [0.0] * 12

        test_msg.effort = [0.0] * 12

        self.policy_joint_pub.publish(test_msg)

        self.get_logger().info("PUBLISHED TEST JOINT STATE")

        # ----------------------------------------------------
        # Subscribers
        # ----------------------------------------------------

        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10,
        )

        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10,
        )

        # ----------------------------------------------------
        # State Buffers
        # ----------------------------------------------------

        self.latest_cmd_msg: Optional[TwistStamped] = None

        self.latest_joint_state_msg: Optional[JointState] = None

        self.latest_imu_msg: Optional[Imu] = None

        # ----------------------------------------------------
        # Previous Action
        # ----------------------------------------------------

        self.previous_action = np.zeros(12, dtype=np.float32)

        # ----------------------------------------------------
        # Joint Mapping
        # ----------------------------------------------------

        self.live_to_policy_index = [
            LIVE_JOINT_ORDER.index(name) for name in POLICY_JOINT_ORDER
        ]

        # ----------------------------------------------------
        # Timer
        # ----------------------------------------------------

        self.timer = self.create_timer(
            0.02,
            self.control_loop,
        )

        self.get_logger().info("Policy controller initialized")

    # ========================================================
    # Callbacks
    # ========================================================

    def cmd_vel_callback(self, msg: TwistStamped):

        self.latest_cmd_msg = msg

    def joint_state_callback(self, msg: JointState):

        self.latest_joint_state_msg = msg

    def imu_callback(self, msg: Imu):

        self.latest_imu_msg = msg

    # ========================================================
    # Observation Construction
    # ========================================================

    def build_observation(self):

        if self.latest_joint_state_msg is None:
            return None

        if self.latest_imu_msg is None:
            return None

        # ----------------------------------------------------
        # Commands
        # ----------------------------------------------------

        if self.latest_cmd_msg is not None:
            cmd_x = self.latest_cmd_msg.twist.linear.x
            cmd_y = self.latest_cmd_msg.twist.linear.y
            cmd_yaw = self.latest_cmd_msg.twist.angular.z

        else:
            cmd_x = 0.0
            cmd_y = 0.0
            cmd_yaw = 0.0

        commands = np.array(
            [
                cmd_x,
                cmd_y,
                cmd_yaw,
            ],
            dtype=np.float32,
        )

        # ----------------------------------------------------
        # IMU
        # ----------------------------------------------------

        imu = self.latest_imu_msg

        quat = [
            imu.orientation.x,
            imu.orientation.y,
            imu.orientation.z,
            imu.orientation.w,
        ]

        rot = R.from_quat(quat)

        gravity_world = np.array([0.0, 0.0, -1.0])

        projected_gravity = rot.inv().apply(gravity_world)

        base_ang_vel = np.array(
            [
                imu.angular_velocity.x,
                imu.angular_velocity.y,
                imu.angular_velocity.z,
            ],
            dtype=np.float32,
        )

        # ----------------------------------------------------
        # Joint States
        # ----------------------------------------------------

        joint_state = self.latest_joint_state_msg

        q = np.array(joint_state.position, dtype=np.float32)

        dq = np.array(joint_state.velocity, dtype=np.float32)

        # remap to policy order

        q = q[self.live_to_policy_index]

        dq = dq[self.live_to_policy_index]

        # normalize around default pose

        q_rel = q - DEFAULT_JOINT_POS

        # ----------------------------------------------------
        # Final Observation
        # ----------------------------------------------------

        base_lin_vel = np.zeros(3, dtype=np.float32)

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

        return obs

    # ========================================================
    # Main Control Loop
    # ========================================================

    def control_loop(self):

        self.get_logger().info("control loop running")

        obs = self.build_observation()

        if obs is None:
            return

        # ----------------------------------------------------
        # Publish Observation Debug
        # ----------------------------------------------------

        obs_msg = Float32MultiArray()

        obs_msg.data = obs.tolist()

        self.policy_obs_pub.publish(obs_msg)

        # ----------------------------------------------------
        # Policy Inference
        # ----------------------------------------------------

        obs_tensor = torch.from_numpy(obs).unsqueeze(0)

        with torch.no_grad():
            action_tensor = self.policy(obs_tensor)

        action = action_tensor.squeeze(0).cpu().numpy()

        self.get_logger().info(f"action shape: {action.shape}")

        # ----------------------------------------------------
        # Convert Action -> Joint Targets
        # ----------------------------------------------------

        target_q = DEFAULT_JOINT_POS + ACTION_SCALE * action

        # ----------------------------------------------------
        # Store Previous Action
        # ----------------------------------------------------

        self.previous_action = action.astype(np.float32)

        # ----------------------------------------------------
        # Publish Low Command
        # ----------------------------------------------------

        lowcmd_msg = Float32MultiArray()

        lowcmd_msg.data = [float(x) for x in target_q]

        self.lowcmd_pub.publish(lowcmd_msg)

        # ----------------------------------------------------
        # Publish Policy Joint States
        # ----------------------------------------------------

        policy_js = JointState()

        policy_js.header.stamp = self.get_clock().now().to_msg()

        policy_js.header.frame_id = "base"

        policy_js.name = POLICY_JOINT_ORDER

        policy_js.position = [float(x) for x in target_q]

        policy_js.velocity = [0.0 for _ in range(12)]

        policy_js.effort = [0.0 for _ in range(12)]

        self.policy_joint_pub.publish(policy_js)

        self.get_logger().info("published policy joint state")

        # ----------------------------------------------------
        # Status
        # ----------------------------------------------------

        status_msg = String()

        status_msg.data = "running"

        self.policy_status_pub.publish(status_msg)


# ============================================================
# Main
# ============================================================


def main(args=None):

    rclpy.init(args=args)

    node = PolicyControllerNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
