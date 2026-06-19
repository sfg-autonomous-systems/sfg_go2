#!/usr/bin/env python3

import argparse
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


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


class SingleJointLowCmdTest(Node):
    def __init__(self, motor_index: int, offset_rad: float, duration_sec: float, rate_hz: float):
        super().__init__("single_joint_lowcmd_test")

        self.motor_index = motor_index
        self.offset_rad = offset_rad
        self.duration_sec = duration_sec
        self.rate_hz = rate_hz

        self.latest_joint_state = None

        self.sub = self.create_subscription(
            JointState,
            "/local/sfg_go2_01/joint_states",
            self.joint_state_callback,
            10,
        )

        self.pub = self.create_publisher(
            Float32MultiArray,
            "/go2/policy_lowcmd",
            10,
        )

    def joint_state_callback(self, msg: JointState):
        self.latest_joint_state = msg

    def wait_for_joint_state(self, timeout_sec: float = 5.0):
        start = time.time()

        while rclpy.ok() and time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.latest_joint_state is not None:
                if len(self.latest_joint_state.position) >= 12:
                    return True

        return False

    def get_current_positions_in_unitree_order(self):
        msg = self.latest_joint_state

        name_to_position = {}

        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                name_to_position[name] = float(msg.position[i])

        missing = [name for name in UNITREE_MOTOR_ORDER if name not in name_to_position]

        if missing:
            raise RuntimeError(f"Missing joints from joint_states: {missing}")

        return [name_to_position[name] for name in UNITREE_MOTOR_ORDER]

    def run_test(self):
        if self.motor_index < 0 or self.motor_index >= 12:
            raise ValueError("motor_index must be between 0 and 11")

        self.get_logger().info("Waiting for /local/sfg_go2_01/joint_states...")

        if not self.wait_for_joint_state():
            raise RuntimeError("Timed out waiting for joint_states")

        initial_q = self.get_current_positions_in_unitree_order()

        target_q = list(initial_q)
        target_q[self.motor_index] = initial_q[self.motor_index] + self.offset_rad

        joint_name = UNITREE_MOTOR_ORDER[self.motor_index]

        self.get_logger().info("====================================================")
        self.get_logger().info("Single Joint LowCmd Test")
        self.get_logger().info(f"Motor index: {self.motor_index}")
        self.get_logger().info(f"Joint name:   {joint_name}")
        self.get_logger().info(f"Initial q:    {initial_q[self.motor_index]:.4f} rad")
        self.get_logger().info(f"Target q:     {target_q[self.motor_index]:.4f} rad")
        self.get_logger().info(f"Offset:       {self.offset_rad:.4f} rad")
        self.get_logger().info(f"Duration:     {self.duration_sec:.2f} sec")
        self.get_logger().info("All other joints commanded to hold initial position.")
        self.get_logger().info("====================================================")

        msg = Float32MultiArray()
        msg.data = [float(x) for x in target_q]

        period = 1.0 / self.rate_hz
        start = time.time()
        next_print = start

        while rclpy.ok() and time.time() - start < self.duration_sec:
            self.pub.publish(msg)

            rclpy.spin_once(self, timeout_sec=0.0)

            now = time.time()

            if now >= next_print:
                current_q = self.get_current_positions_in_unitree_order()
                current_motor_q = current_q[self.motor_index]

                self.get_logger().info(
                    f"motor {self.motor_index} {joint_name}: "
                    f"current={current_motor_q:.4f}, "
                    f"target={target_q[self.motor_index]:.4f}, "
                    f"error={target_q[self.motor_index] - current_motor_q:.4f}"
                )

                next_print = now + 0.5

            time.sleep(period)

        final_q = self.get_current_positions_in_unitree_order()

        self.get_logger().info("====================================================")
        self.get_logger().info("Test complete.")
        self.get_logger().info(f"Initial q: {initial_q[self.motor_index]:.4f}")
        self.get_logger().info(f"Final q:   {final_q[self.motor_index]:.4f}")
        self.get_logger().info(f"Delta:     {final_q[self.motor_index] - initial_q[self.motor_index]:.4f}")
        self.get_logger().info("====================================================")


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--motor",
        type=int,
        default=0,
        help="Motor index in Unitree order, 0-11. Default: 0 front_right_hip_joint.",
    )

    parser.add_argument(
        "--offset",
        type=float,
        default=0.15,
        help="Target offset in radians from current position. Default: +0.15 rad.",
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=3.0,
        help="Duration in seconds. Default: 3.0.",
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=50.0,
        help="Publish rate in Hz. Default: 50.",
    )

    args = parser.parse_args()

    rclpy.init()

    node = SingleJointLowCmdTest(
        motor_index=args.motor,
        offset_rad=args.offset,
        duration_sec=args.duration,
        rate_hz=args.rate,
    )

    try:
        node.run_test()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
