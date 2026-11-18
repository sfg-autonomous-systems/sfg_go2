import launch
import sfg_utils.launch_utils
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from rospkg import get_package_name
from sfg_utils.fqn import (
    Component,
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

    wireless_fqn_builder = global_namespace.component(
        Component.Custom, "wireless_state_publisher"
    )
    wireless_state_publisher_node = ComposableNode(
        package=package_name,
        plugin="sfg_go2_hardware_interface::WirelessStatePublisher",
        namespace=local_namespace.build(RosFqnSegment.Scope, RosFqnSegment.Agent),
        name=wireless_fqn_builder.build(RosFqnSegment.Component),
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "config",
                    "wireless_state_publisher.yaml",
                ]
            ),
            {"map_file": LaunchConfiguration("qos_map_file")},
        ],
    )
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "qos_map_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare(package_name),
                        "config",
                        "heatmap_example.csv",
                    ]
                ),
                description="Absolute path to the saved QoS foundation CSV map.",
            ),
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
                    *wireless_state_publisher_node,
                ],
            ),
        ]
    )
