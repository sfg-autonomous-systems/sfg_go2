from typing import Any

import launch
import sfg_utils.launch_utils
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from rospkg import get_package_name
from sfg_utils.fqn import (
    RosFQNBuilder,
    RosFQNSegment,
    Scope,
)

package_name = get_package_name(__file__)


def get_nodes(
    local_namespace: str, global_namespace: str, **arguments: Any
) -> tuple[list[Node], list[ComposableNode]]:
    base_platform_nodes, base_platform_composable_nodes = (
        sfg_utils.launch_utils.get_nodes(
            local_namespace, global_namespace, package_name, "base_platform.launch.py"
        )
    )
    payload_platform_nodes, payload_platform_composable_nodes = (
        sfg_utils.launch_utils.get_nodes(
            local_namespace,
            global_namespace,
            package_name,
            "payload_platform.launch.py",
        )
    )

    return (
        base_platform_nodes + payload_platform_nodes,
        base_platform_composable_nodes + payload_platform_composable_nodes,
    )


def generate_launch_description() -> launch.LaunchDescription:
    local_namespace, global_namespace = (
        RosFQNBuilder()
        .scope(Scope.Local)
        .agent()
        .build(begin=RosFQNSegment.Scope, end=RosFQNSegment.Agent),
        RosFQNBuilder()
        .scope(Scope.Global)
        .agent()
        .build(begin=RosFQNSegment.Scope, end=RosFQNSegment.Agent),
    )

    nodes, composable_nodes = get_nodes(
        local_namespace,
        global_namespace,
    )

    return launch.LaunchDescription(
        [
            *nodes,
            ComposableNodeContainer(
                package="rclcpp_components",
                executable="component_container_mt",
                namespace=local_namespace,
                name="hardware_interface_container",
                output="screen",
                composable_node_descriptions=composable_nodes,
            ),
        ]
    )
