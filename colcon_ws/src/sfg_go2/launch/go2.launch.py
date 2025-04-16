import socket
from pathlib import Path

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

package_directory = Path(get_package_share_directory("sfg_go2"))
local_namespace = "/local/" + socket.gethostname().replace("-", "_")
global_namespace = "/global/" + socket.gethostname().replace("-", "_")


def generate_launch_description():
    heartbeat_node = Node(
        package="sfg_heartbeat",
        executable="heartbeat",
        namespace=local_namespace,
        name="heartbeat",
        parameters=[
            package_directory / "config" / "heartbeat.yml",
        ],
        remappings=[
            ("heartbeat", "/global/heartbeat"),
        ],
        output="screen",
    )

    jtop_diagnostics_group = GroupAction(
        actions=[
            Node(
                package="isaac_ros_jetson_stats",
                executable="jtop",
                name="jtop",
                parameters=[package_directory / "config" / "jtop.yml"],
                output="screen",
            ),
            Node(
                package="diagnostic_aggregator",
                executable="aggregator_node",
                parameters=[
                    Path(get_package_share_directory("isaac_ros_jetson_stats"))
                    / "config"
                    / "jtop.yaml",
                ],
                output="screen",
            ),
        ]
    )

    camera_head_container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_mt",
        namespace=local_namespace,
        name="camera_head_container",
        composable_node_descriptions=(
            ComposableNode(
                package="realsense2_camera",
                plugin="realsense2_camera::RealSenseNodeFactory",
                namespace=local_namespace,
                name="camera_head",
                parameters=[
                    package_directory / "config" / "camera_head.yml",
                ],
            ),
            ComposableNode(
                package="isaac_ros_h264_encoder",
                plugin="nvidia::isaac_ros::h264_encoder::EncoderNode",
                namespace=local_namespace,
                name="camera_head_encoder",
                remappings=[
                    ("image_raw", local_namespace + "/camera_head/color/image_raw"),
                    (
                        "image_compressed",
                        global_namespace + "/camera_head/color_compressed",
                    ),
                ],
            ),
        ),
        output="screen",
    )

    return launch.LaunchDescription(
        [
            heartbeat_node,
            jtop_diagnostics_group,
            camera_head_container,
        ]
    )
