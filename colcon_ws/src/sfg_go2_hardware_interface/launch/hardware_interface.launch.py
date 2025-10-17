import launch
import sfg_utils.launch_utils
from launch_ros.actions import ComposableNodeContainer
from rospkg import get_package_name
from sfg_utils.fqn import (
    RosFqnBuilder,
    RosFqnSegment,
    Scope,
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

    return launch.LaunchDescription(
        [
            *base_platform_launch_description_entities.launch_arguments,
            *payload_platform_launch_description_entities.launch_arguments,
            *base_platform_launch_description_entities.nodes,
            *payload_platform_launch_description_entities.nodes,
            ComposableNodeContainer(
                package="rclcpp_components",
                executable="component_container_mt",
                namespace=local_namespace,
                name="hardware_interface_container",
                output="screen",
                composable_node_descriptions=base_platform_launch_description_entities.composable_nodes
                + payload_platform_launch_description_entities.composable_nodes,
            ),
        ]
    )
