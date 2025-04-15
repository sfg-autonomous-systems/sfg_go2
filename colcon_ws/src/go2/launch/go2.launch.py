import socket
from pathlib import Path

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

package_directory = Path(get_package_share_directory("go2"))
local_namespace = "/local/" + socket.gethostname().replace("-", "_")
global_namespace = "/global/" + socket.gethostname().replace("-", "_")


def generate_launch_description():
    jtop_diagnostics_group = GroupAction(
        actions=[
            Node(
                package="isaac_ros_jetson_stats",
                name="jtop",
                executable="jtop",
                output="screen",
                parameters=[package_directory / "config" / "jtop.yml"],
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
        name="camera_head_container",
        namespace=local_namespace,
        executable="component_container_mt",
        composable_node_descriptions=(
            ComposableNode(
                name="camera_head",
                namespace=local_namespace,
                package="realsense2_camera",
                plugin="realsense2_camera::RealSenseNodeFactory",
                parameters=[
                    package_directory / "config" / "camera_head.yml",
                ],
            ),
            ComposableNode(
                name="camera_head_encoder",
                package="isaac_ros_h264_encoder",
                plugin="nvidia::isaac_ros::h264_encoder::EncoderNode",
                namespace=local_namespace,
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

    return launch.LaunchDescription([jtop_diagnostics_group, camera_head_container])
