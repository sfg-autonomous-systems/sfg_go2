#!/usr/bin/env python3

import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray, String


class PolicyControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("policy_controller_node")

        # Parameters
        self.control_rate_hz = float(
            self.declare_parameter("control_rate_hz", 50.0).value
        )
        self.cmd_timeout_sec = float(
            self.declare_parameter("cmd_timeout_sec", 0.25).value
        )
        self.state_timeout_sec = float(
            self.declare_parameter("state_timeout_sec", 0.25).value
        )
        self.publish_debug_topics = bool(
            self.declare_parameter("publish_debug_topics", True).value
        )

        self.cmd_vel_topic = str(
            self.declare_parameter(
                "cmd_vel_topic",
                "/global/sfg_go2_01/locomotion_controller/cmd_vel",
            ).value
        )
        self.imu_topic = str(self.declare_parameter("imu_topic", "/go2/imu").value)
        self.joint_state_topic = str(
            self.declare_parameter(
                "joint_state_topic", "/local/sfg_go2_01/joint_states"
            ).value
        )

        # Policy contract
        self.num_actions = 12
        self.num_obs = 48

        # Training default stance from your Isaac Lab env
        self.default_q = np.array(
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

        # IMPORTANT:
        # Your live joint_states are currently ordered:
        # FR, FL, RR, RL
        # but your policy likely expects:
        # FL, FR, RL, RR
        #
        # This mapping converts live joint_states order -> policy order.
        self.live_joint_order = [
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

        self.policy_joint_order = [
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

        self.live_to_policy_index = [
            self.live_joint_order.index(name) for name in self.policy_joint_order
        ]

        # State buffers
        self.latest_cmd_msg: Optional[TwistStamped] = None
        self.latest_cmd_time: Optional[Time] = None

        self.latest_imu_msg: Optional[Imu] = None
        self.latest_imu_time: Optional[Time] = None

        self.latest_joint_state_msg: Optional[JointState] = None
        self.latest_joint_state_time: Optional[Time] = None

        self.prev_action = np.zeros(self.num_actions, dtype=np.float32)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )

        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10,
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10,
        )

        # Publishers
        self.policy_lowcmd_pub = self.create_publisher(
            Float32MultiArray, "/go2/policy_lowcmd", 10
        )

        self.status_pub = self.create_publisher(String, "/go2/policy_status", 10)
        self.obs_pub = self.create_publisher(Float32MultiArray, "/go2/policy_obs", 10)
        self.action_pub = self.create_publisher(
            Float32MultiArray, "/go2/policy_action", 10
        )

        # Timer
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_loop)

        self.get_logger().info("Started policy_controller_node.")
        self.get_logger().info(f"Subscribing cmd_vel to: {self.cmd_vel_topic}")
        self.get_logger().info(f"Subscribing imu to: {self.imu_topic}")
        self.get_logger().info(f"Subscribing joint_states to: {self.joint_state_topic}")

    # ---------------------------
    # Callbacks
    # ---------------------------

    def cmd_vel_callback(self, msg: TwistStamped) -> None:
        self.latest_cmd_msg = msg
        self.latest_cmd_time = self.get_clock().now()

    def imu_callback(self, msg: Imu) -> None:
        self.latest_imu_msg = msg
        self.latest_imu_time = self.get_clock().now()

    def joint_state_callback(self, msg: JointState) -> None:
        self.latest_joint_state_msg = msg
        self.latest_joint_state_time = self.get_clock().now()

    # ---------------------------
    # Main control loop
    # ---------------------------

    def control_loop(self) -> None:
        status = self.compute_status()
        self.publish_status(status)

        if status != "running":
            return

        obs = self.build_observation()

        # Stage 1 dummy policy:
        # publish a safe 12-element zero vector.
        action = np.zeros(self.num_actions, dtype=np.float32)

        self.publish_policy_lowcmd(action)

        if self.publish_debug_topics:
            self.publish_array(self.obs_pub, obs)
            self.publish_array(self.action_pub, action)

        self.prev_action = action.copy()

    # ---------------------------
    # Status / safety
    # ---------------------------

    def compute_status(self) -> str:
        now = self.get_clock().now()

        if self.latest_imu_time is None:
            return "waiting_for_imu"
        if self.latest_joint_state_time is None:
            return "waiting_for_joint_states"

        if (now - self.latest_imu_time).nanoseconds * 1e-9 > self.state_timeout_sec:
            return "stale_imu"
        if (
            now - self.latest_joint_state_time
        ).nanoseconds * 1e-9 > self.state_timeout_sec:
            return "stale_joint_states"

        # cmd_vel is allowed to go stale; we just zero it if it does.
        return "running"

    def publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    # ---------------------------
    # Observation building
    # ---------------------------

    def build_observation(self) -> np.ndarray:
        base_lin_vel = self.get_base_lin_vel_estimate()
        base_ang_vel = self.get_base_ang_vel()
        projected_gravity = self.get_projected_gravity()
        velocity_commands = self.get_cmd_vel()
        joint_pos_rel = self.get_joint_pos_rel()
        joint_vel_rel = self.get_joint_vel_rel()
        last_action = self.prev_action.copy()

        obs = np.concatenate(
            [
                base_lin_vel,  # 3
                base_ang_vel,  # 3
                projected_gravity,  # 3
                velocity_commands,  # 3
                joint_pos_rel,  # 12
                joint_vel_rel,  # 12
                last_action,  # 12
            ],
            dtype=np.float32,
        )

        if obs.shape[0] != self.num_obs:
            raise RuntimeError(
                f"Observation has wrong size {obs.shape[0]}, expected {self.num_obs}"
            )

        return obs

    def get_base_lin_vel_estimate(self) -> np.ndarray:
        # Placeholder for now.
        # Your policy expects this term, but you do not currently have
        # a trustworthy estimator topic in the stack.
        return np.zeros(3, dtype=np.float32)

    def get_base_ang_vel(self) -> np.ndarray:
        assert self.latest_imu_msg is not None
        msg = self.latest_imu_msg
        return np.array(
            [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ],
            dtype=np.float32,
        )

    def get_projected_gravity(self) -> np.ndarray:
        assert self.latest_imu_msg is not None
        q = self.latest_imu_msg.orientation
        quat_xyzw = np.array([q.x, q.y, q.z, q.w], dtype=np.float32)
        return self.gravity_in_body_frame(quat_xyzw)

    def get_cmd_vel(self) -> np.ndarray:
        if self.latest_cmd_msg is None or self.latest_cmd_time is None:
            return np.zeros(3, dtype=np.float32)

        age_sec = (self.get_clock().now() - self.latest_cmd_time).nanoseconds * 1e-9
        if age_sec > self.cmd_timeout_sec:
            return np.zeros(3, dtype=np.float32)

        msg = self.latest_cmd_msg
        return np.array(
            [
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.angular.z,
            ],
            dtype=np.float32,
        )

    def get_joint_pos_rel(self) -> np.ndarray:
        q, _ = self.get_policy_order_joint_state()
        return q - self.default_q

    def get_joint_vel_rel(self) -> np.ndarray:
        _, dq = self.get_policy_order_joint_state()
        return dq

    def get_policy_order_joint_state(self) -> tuple[np.ndarray, np.ndarray]:
        assert self.latest_joint_state_msg is not None
        msg = self.latest_joint_state_msg

        if len(msg.position) < self.num_actions or len(msg.velocity) < self.num_actions:
            raise RuntimeError(
                "JointState does not contain 12 positions and velocities."
            )

        q_live = np.array(msg.position[: self.num_actions], dtype=np.float32)
        dq_live = np.array(msg.velocity[: self.num_actions], dtype=np.float32)

        q_policy = q_live[self.live_to_policy_index]
        dq_policy = dq_live[self.live_to_policy_index]

        return q_policy, dq_policy

    # ---------------------------
    # Math helpers
    # ---------------------------

    def gravity_in_body_frame(self, quat_xyzw: np.ndarray) -> np.ndarray:
        # quat expected in [x, y, z, w]
        x, y, z, w = quat_xyzw

        # Rotation matrix body->world from quaternion
        r00 = 1.0 - 2.0 * (y * y + z * z)
        r01 = 2.0 * (x * y - z * w)
        r02 = 2.0 * (x * z + y * w)

        r10 = 2.0 * (x * y + z * w)
        r11 = 1.0 - 2.0 * (x * x + z * z)
        r12 = 2.0 * (y * z - x * w)

        r20 = 2.0 * (x * z - y * w)
        r21 = 2.0 * (y * z + x * w)
        r22 = 1.0 - 2.0 * (x * x + y * y)

        R = np.array(
            [
                [r00, r01, r02],
                [r10, r11, r12],
                [r20, r21, r22],
            ],
            dtype=np.float32,
        )

        # World gravity in world frame
        g_world = np.array([0.0, 0.0, -1.0], dtype=np.float32)

        # projected gravity in body frame = R^T * g_world
        g_body = R.T @ g_world
        return g_body.astype(np.float32)

    # ---------------------------
    # Publishing
    # ---------------------------

    def publish_policy_lowcmd(self, action: np.ndarray) -> None:
        msg = Float32MultiArray()
        msg.data = action.astype(np.float32).tolist()
        self.policy_lowcmd_pub.publish(msg)

    def publish_array(self, publisher, array: np.ndarray) -> None:
        msg = Float32MultiArray()
        msg.data = array.astype(np.float32).tolist()
        publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PolicyControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
