import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from rospkg import get_package_name
from sfg_utils import get_ros_namespaces

package_name = get_package_name(__file__)
local_namespace, global_namespace = get_ros_namespaces()


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
                    {
                        "metadata_filepath": PathJoinSubstitution(
                            [
                                FindPackageShare(package_name),
                                "config",
                                "agent_metadata.yaml",
                            ]
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
                    PathJoinSubstitution(
                        [FindPackageShare(package_name), "config", "camera_head.yaml"]
                    )
                ],
                remappings=[
                    # Remap depth related topics to the global namespace.
                    # Note that the depth shared via the global namespace is already
                    # the one that is aligned to the color image even though the
                    # topic name does not explicitly mention it.
                    (
                        "camera_head/aligned_depth_to_color/image_raw/compressedDepth",
                        f"{global_namespace}/camera_head/depth/image_compressed",
                    ),
                    (
                        "camera_head/aligned_depth_to_color/camera_info",
                        f"{global_namespace}/camera_head/depth/camera_info",
                    ),
                    # Remap color related topics to the global namespace.
                    (
                        "camera_head/color/image_raw/ffmpeg",
                        f"{global_namespace}/camera_head/color/image_compressed",
                    ),
                    (
                        "camera_head/color/camera_info",
                        f"{global_namespace}/camera_head/color/camera_info",
                    ),
                ],
                # We do not use intra-process communication here because for some reason
                # not all of the image_transport plugins work if enabled.
            ),
            ComposableNode(
                package="livox_ros_driver2",
                plugin="livox_ros::DriverNode",
                namespace=local_namespace,
                name="lidar_back",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare(package_name), "config", "lidar_back.yaml"]
                    ),
                    {
                        "user_config_path": (
                            PathJoinSubstitution(
                                [
                                    FindPackageShare(package_name),
                                    "config",
                                    "lidar_back_config.json",
                                ]
                            )
                        ),
                    },
                ],
                remappings=[
                    ("livox/imu", f"{global_namespace}/lidar_back/imu"),
                    ("livox/lidar", f"{global_namespace}/lidar_back/points"),
                ],
            ),
            ComposableNode(
                package="sfg_go2_hardware_interface",
                plugin="sfg_go2_hardware_interface::LocomotionController",
                namespace=local_namespace,
                name="locomotion_controller",
                parameters=[
                    (
                        PathJoinSubstitution(
                            [
                                FindPackageShare("sfg_go2_hardware_interface"),
                                "config",
                                "locomotion_controller.yaml",
                            ]
                        )
                    ),
                ],
                remappings=[
                    (
                        "locomotion_controller/cmd_vel",
                        global_namespace + "/locomotion_controller/cmd_vel",
                    ),
                    (
                        "locomotion_controller/set_locomotion_mode",
                        global_namespace + "/locomotion_controller/set_locomotion_mode",
                    ),
                ],
            ),
        ),
        output="screen",
    )

    lighthouse_tracker_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("sfg_lighthouse_tracking"),
                        "launch",
                        "lighthouse_tracker.launch.py",
                    ]
                )
            ]
        )
    )

    return launch.LaunchDescription(
        [
            jtop_launch_description,
            go2_container,
            lighthouse_tracker_launch_description,
        ]
    )
