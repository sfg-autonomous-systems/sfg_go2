import launch
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from rospkg import get_package_name
from sfg_utils.fqn import (
    Component,
    Resource,
    RosFqnBuilder,
    RosFqnSegment,
    Scope,
    Stream,
)

package_name = get_package_name(__file__)
local_namespace, global_namespace = (
    RosFqnBuilder()
    .scope(Scope.Local)
    .agent()
    .build(begin=RosFqnSegment.Scope, end=RosFqnSegment.Agent),
    RosFqnBuilder()
    .scope(Scope.Global)
    .agent()
    .build(begin=RosFqnSegment.Scope, end=RosFqnSegment.Agent),
)


def generate_launch_description() -> launch.LaunchDescription:
    camera_head_fqn_builder = (
        RosFqnBuilder().scope(Scope.Global).agent().component(Component.Camera, "head")
    )
    camera_head_name = camera_head_fqn_builder.build(RosFqnSegment.Component)
    camera_head_node = ComposableNode(
        package="realsense2_camera",
        plugin="realsense2_camera::RealSenseNodeFactory",
        namespace=local_namespace,
        name=camera_head_name,
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "config",
                    f"{camera_head_name}.yaml",
                ]
            ),
            {
                "camera_name": f"{camera_head_fqn_builder.build(begin=RosFqnSegment.Agent, end=RosFqnSegment.Component)}"
            },
        ],
        remappings=[
            # Remap depth related topics to the global namespace.
            # Note that the depth shared via the global namespace is already
            # the one that is aligned to the color image even though the
            # topic name does not explicitly mention it.
            (
                f"{camera_head_name}/aligned_depth_to_color/image_raw/compressedDepth",
                camera_head_fqn_builder.stream(Stream.Depth)
                .resource(Resource.ImageCompressed)
                .build(),
            ),
            (
                f"{camera_head_name}/aligned_depth_to_color/camera_info",
                camera_head_fqn_builder.resource(Resource.CameraInfo).build(),
            ),
            # Remap color related topics to the global namespace.
            (
                f"{camera_head_name}/color/image_raw/ffmpeg",
                camera_head_fqn_builder.stream(Stream.Color)
                .resource(Resource.ImageCompressed)
                .build(),
            ),
            (
                f"{camera_head_name}/color/camera_info",
                camera_head_fqn_builder.resource(Resource.CameraInfo).build(),
            ),
        ],
        # ToDo: We do not use intra-process communication here because for some reason
        # not all of the image_transport plugins work if enabled.
    )

    lidar_back_fqn_builder = (
        RosFqnBuilder().scope(Scope.Global).agent().component(Component.Lidar, "back")
    )
    lidar_back_name = lidar_back_fqn_builder.build(RosFqnSegment.Component)
    lidar_back_node = ComposableNode(
        package="livox_ros_driver2",
        plugin="livox_ros::DriverNode",
        namespace=local_namespace,
        name=lidar_back_name,
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "config",
                    f"{lidar_back_name}.yaml",
                ]
            ),
            {
                "user_config_path": (
                    PathJoinSubstitution(
                        [
                            FindPackageShare(package_name),
                            "config",
                            f"{lidar_back_name}_config.json",
                        ]
                    )
                ),
                "point_cloud_frame_id": f"{lidar_back_fqn_builder.build(begin=RosFqnSegment.Agent, end=RosFqnSegment.Component)}_point_cloud_frame",
                "imu_frame_id": f"{lidar_back_fqn_builder.build(begin=RosFqnSegment.Agent, end=RosFqnSegment.Component)}_imu_frame",
            },
        ],
        remappings=[
            (
                "livox/imu",
                lidar_back_fqn_builder.resource(Resource.Imu).build(),
            ),
            (
                "livox/lidar",
                lidar_back_fqn_builder.resource(Resource.PointCloud).build(),
            ),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    return launch.LaunchDescription(
        [
            ComposableNodeContainer(
                package="rclcpp_components",
                executable="component_container_mt",
                namespace=local_namespace,
                name="payload_platform_container",
                output="screen",
                composable_node_descriptions=[
                    camera_head_node,
                    lidar_back_node,
                ],
            ),
        ]
    )
