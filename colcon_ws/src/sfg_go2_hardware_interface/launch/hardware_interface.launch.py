import launch
import sfg_utils.launch_utils
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from rospkg import get_package_name
from sfg_utils.fqn import (
    RosFqnBuilder,
    RosFqnSegment,
    Scope,
)

package_name = get_package_name(__file__)
local_namespace = RosFqnBuilder().scope(Scope.Local).agent()
global_namespace = RosFqnBuilder().scope(Scope.Global).agent()


def generate_launch_description() -> launch.LaunchDescription:
    base_platform_launch_description_entities = (
        sfg_utils.launch_utils.get_launch_description_entities(
            package_name, "base_platform.launch.py"
        )
    )
    payload_platform_launch_description_entities = (
        sfg_utils.launch_utils.get_launch_description_entities(
            package_name, "payload_platform.launch.py"
        )
    )

    vicon_localizer_node = ComposableNode(
        package="sfg_hardware_interface",
        plugin="sfg_hardware_interface::ViconLocalizer",
        namespace=local_namespace.build(RosFqnSegment.Scope, RosFqnSegment.Agent),
        name="vicon_localizer",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "config",
                    "vicon_localizer.yaml",
                ]
            ),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    return launch.LaunchDescription(
        [
            *base_platform_launch_description_entities.launch_arguments,
            *payload_platform_launch_description_entities.launch_arguments,
            *base_platform_launch_description_entities.nodes,
            *payload_platform_launch_description_entities.nodes,
            ComposableNodeContainer(
                package="rclcpp_components",
                executable="component_container_mt",
                namespace=local_namespace.build(
                    RosFqnSegment.Scope, RosFqnSegment.Agent
                ),
                name="hardware_interface_container",
                output="screen",
                composable_node_descriptions=[
                    *base_platform_launch_description_entities.composable_nodes,
                    *payload_platform_launch_description_entities.composable_nodes,
                    vicon_localizer_node,
                ],
            ),
        ]
    )
