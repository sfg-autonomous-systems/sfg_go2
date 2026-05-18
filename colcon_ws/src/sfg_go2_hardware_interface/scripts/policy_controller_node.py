#!/usr/bin/env python3

import numpy as np
import rclpy
import torch
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray, String

# ============================================================
# POLICY CONFIG
# ============================================================

POLICY_PATH = (
    "/workspace/go2/colcon_ws/src/sfg_go2_hardware_interface/scripts/policy.pt"
)

ACTION_SCALE = 0.25

# ============================================================
# JOINT ORDER
# ============================================================

# IMPORTANT:
# These names MUST match:
#   /local/sfg_go2_01/joint_states
#
# Verify with:
#
# ros2 topic echo /local/sfg_go2_01/joint_states --once
#
# and check msg.name
#
# Then update if necessary.

JOINT_ORDER = [
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
# DEFAULT JOINT POSE
# ============================================================

# IsaacLab default standing pose

DEFAULT_JOINT_POS = np.array(
    [
        -0.1,
        0.8,
        -1.5,
        0.1,
        0.8,
        -1.5,
        -0.1,
        1.0,
        -1.5,
        0.1,
        1.0,
        -1.5,
    ],
    dtype=np.float32,
)

# ============================================================
# POLICY NODE
# ============================================================


class PolicyControllerNode(Node):
    def __init__(self) -> None:

        super().__init__("policy_controller")

        self.get_logger().info("Starting Go2 policy controller")

        # ====================================================
        # Load TorchScript Policy
        # ====================================================

        self.policy = torch.jit.load(POLICY_PATH)
        self.policy.eval()

        self.get_logger().info(f"Loaded policy: {POLICY_PATH}")

        # ====================================================
        # Topic Names
        # ====================================================

        self.cmd_vel_topic = "/global/sfg_go2_01/locomotion_controller/cmd_vel"

        self.imu_topic = "/go2/imu"

        self.joint_state_topic = "/local/sfg_go2_01/joint_states"

        # ====================================================
        # Internal State Buffers
        # ====================================================

        self.base_lin_vel = np.zeros(3, dtype=np.float32)

        self.base_ang_vel = np.zeros(3, dtype=np.float32)

        self.projected_gravity = np.array(
            [0.0, 0.0, -1.0],
            dtype=np.float32,
        )

        self.commands = np.zeros(3, dtype=np.float32)

        self.joint_pos = np.zeros(12, dtype=np.float32)

        self.joint_vel = np.zeros(12, dtype=np.float32)

        self.previous_action = np.zeros(12, dtype=np.float32)

        # ====================================================
        # Subscribers
        # ====================================================

        self.cmd_sub = self.create_subscription(
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

        # ====================================================
        # Publishers
        # ====================================================

        self.policy_action_pub = self.create_publisher(
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

        # ====================================================
        # Control Timer
        # ====================================================

        self.control_timer = self.create_timer(
            0.02,
            self.control_loop,
        )

        self.get_logger().info("Started policy controller node.")
        self.get_logger().info(f"Subscribing cmd_vel to: {self.cmd_vel_topic}")
        self.get_logger().info(f"Subscribing imu to: {self.imu_topic}")
        self.get_logger().info(f"Subscribing joint_states to: {self.joint_state_topic}")

    # ========================================================
    # Callbacks
    # ========================================================

    def cmd_vel_callback(
        self,
        msg: TwistStamped,
    ) -> None:

        self.commands[0] = msg.twist.linear.x
        self.commands[1] = msg.twist.linear.y
        self.commands[2] = msg.twist.angular.z

    def imu_callback(
        self,
        msg: Imu,
    ) -> None:

        # --------------------------------------------
        # Angular velocity
        # --------------------------------------------

        self.base_ang_vel[0] = msg.angular_velocity.x
        self.base_ang_vel[1] = msg.angular_velocity.y
        self.base_ang_vel[2] = msg.angular_velocity.z

        # --------------------------------------------
        # Quaternion
        # --------------------------------------------

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        rotation = R.from_quat([qx, qy, qz, qw])

        # --------------------------------------------
        # Projected gravity
        # --------------------------------------------

        gravity_world = np.array([0.0, 0.0, -1.0])

        gravity_body = rotation.inv().apply(gravity_world)

        self.projected_gravity = gravity_body.astype(np.float32)

    def joint_state_callback(
        self,
        msg: JointState,
    ) -> None:

        name_to_index = {name: i for i, name in enumerate(msg.name)}

        for i, joint_name in enumerate(JOINT_ORDER):
            if joint_name not in name_to_index:
                continue

            idx = name_to_index[joint_name]

            self.joint_pos[i] = msg.position[idx]
            self.joint_vel[i] = msg.velocity[idx]

    # ========================================================
    # Observation Construction
    # ========================================================

    def build_observation(self) -> np.ndarray:

        joint_pos_rel = self.joint_pos - DEFAULT_JOINT_POS

        obs = np.concatenate(
            [
                # 0:3
                self.base_lin_vel,
                # 3:6
                self.base_ang_vel,
                # 6:9
                self.projected_gravity,
                # 9:12
                self.commands,
                # 12:24
                joint_pos_rel,
                # 24:36
                self.joint_vel,
                # 36:48
                self.previous_action,
            ]
        ).astype(np.float32)

        return obs

    # ========================================================
    # Main Control Loop
    # ========================================================

    def control_loop(self) -> None:

        # ----------------------------------------------------
        # Build observation
        # ----------------------------------------------------

        obs = self.build_observation()

        # ----------------------------------------------------
        # Publish observation for debugging
        # ----------------------------------------------------

        obs_msg = Float32MultiArray()
        obs_msg.data = obs.tolist()

        self.policy_obs_pub.publish(obs_msg)

        # ----------------------------------------------------
        # Run policy
        # ----------------------------------------------------

        obs_tensor = torch.tensor(
            obs,
            dtype=torch.float32,
        ).unsqueeze(0)

        with torch.no_grad():
            action_tensor = self.policy(obs_tensor)

        action = action_tensor.squeeze(0).cpu().numpy()

        # ----------------------------------------------------
        # Store previous action
        # ----------------------------------------------------

        self.previous_action = action.copy()

        # ----------------------------------------------------
        # Publish raw policy output
        # ----------------------------------------------------

        action_msg = Float32MultiArray()
        action_msg.data = action.tolist()

        self.policy_action_pub.publish(action_msg)

        # ----------------------------------------------------
        # Publish status
        # ----------------------------------------------------

        status_msg = String()
        status_msg.data = "running"

        self.policy_status_pub.publish(status_msg)

    # ========================================================
    # Future Function
    # ========================================================

    # FUTURE:
    #
    # Later we will convert:
    #
    # target_q =
    #   DEFAULT_JOINT_POS
    #   + ACTION_SCALE * action
    #
    # into true Unitree LowCmd messages.
    #
    # RIGHT NOW:
    #
    # We are ONLY publishing:
    #
    #   /go2/policy_lowcmd
    #
    # as a debug ROS topic.
    #
    # NO MOTOR COMMANDS ARE BEING SENT.
    #


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
