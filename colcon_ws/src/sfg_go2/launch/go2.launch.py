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
    jtop_diagnostics_group = GroupAction(
        actions=[
            Node(
                package="isaac_ros_jetson_stats",
                executable="jtop",
                name="jtop",
                parameters=[package_directory / "config" / "jtop.yaml"],
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

    go2_container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_mt",
        namespace=local_namespace,
        name="go2_container",
        composable_node_descriptions=(
            ComposableNode(
                package="sfg_agent",
                plugin="sfg_agent::AgentStatusProvider",
                namespace=local_namespace,
                parameters=[
                    package_directory / "config" / "agent_status_provider.yaml",
                    {
                        "metadata_filepath": str(
                            package_directory / "config" / "agent_metadata.yaml"
                        )
                    },
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="realsense2_camera",
                plugin="realsense2_camera::RealSenseNodeFactory",
                namespace=local_namespace,
                name="camera_head",
                parameters=[
                    package_directory / "config" / "camera_head.yaml",
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
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
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ),
        output="screen",
    )

    return launch.LaunchDescription(
        [
            jtop_diagnostics_group,
            go2_container,
        ]
    )
