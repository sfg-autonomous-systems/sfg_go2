import socket
from pathlib import Path

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from sfg_utils import sanitize_hostname

package_directory = Path(get_package_share_directory("sfg_go2"))
sanitized_hostname = sanitize_hostname(socket.gethostname())
local_namespace = "/local/" + sanitized_hostname
global_namespace = "/global/" + sanitized_hostname


def generate_launch_description():
    go2_container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_mt",
        namespace=local_namespace,
        name="go2_container",
        composable_node_descriptions=(
            ComposableNode(
                package="realsense2_camera",
                plugin="realsense2_camera::RealSenseNodeFactory",
                namespace=local_namespace,
                name="camera_head",
                parameters=[
                    package_directory / "config" / "camera_head.yaml",
                ],
            ),
        ),
        output="screen",
    )

    return launch.LaunchDescription(
        [
            go2_container,
        ]
    )
