import socket
from pathlib import Path

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from sfg_utils import sanitize_hostname

package_directory = Path(get_package_share_directory("sfg_go2"))
sanitized_hostname = sanitize_hostname(socket.gethostname())
local_namespace = "/local/" + sanitized_hostname
global_namespace = "/global/" + sanitized_hostname


def generate_launch_description():
    jtop_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("isaac_ros_jetson_stats"),
                        "launch",
                        "jtop.launch.py",
                    ]
                )
            ]
        ),
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
                        "metadata_filepath": (
                            package_directory / "config" / "agent_metadata.yaml"
                        ).as_posix(),
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
                remappings=[
                    # Remap depth related topics to the global namespace.
                    # Note that the depth shared via the global namespace is already
                    # the one that is aligned to the color image even though the
                    # topic name does not explicitly mention it.
                    (
                        "camera_head/aligned_depth_to_color/image_raw/compressedDepth",
                        f"{global_namespace}/camera_head/depth_compressed",
                    ),
                    (
                        "camera_head/aligned_depth_to_color/camera_info",
                        f"{global_namespace}/camera_head/depth/camera_info",
                    ),
                    # Remap color related topics to the global namespace.
                    (
                        "camera_head/color/image_raw/ffmpeg",
                        f"{global_namespace}/camera_head/color_compressed",
                    ),
                    (
                        "camera_head/color/camera_info",
                        f"{global_namespace}/camera_head/color/camera_info",
                    ),
                ],
            ),
            ComposableNode(
                package="livox_ros_driver2",
                plugin="livox_ros::DriverNode",
                namespace=local_namespace,
                name="lidar_back",
                parameters=[
                    package_directory / "config" / "lidar_back.yaml",
                    {
                        "user_config_path": (
                            package_directory / "config" / "lidar_back_config.json"
                        ).as_posix(),
                    },
                ],
                remappings=[
                    ("livox/imu", f"{global_namespace}/lidar_back/imu"),
                    ("livox/lidar", f"{global_namespace}/lidar_back/pcl"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ),
        output="screen",
    )

    return launch.LaunchDescription(
        [
            jtop_launch_description,
            go2_container,
        ]
    )
