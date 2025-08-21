from typing import Any

import launch
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from rospkg import get_package_name
from sfg_utils.fqn import (
    Component,
    Resource,
    RosFQNBuilder,
    RosFQNSegment,
    Scope,
)

package_name = get_package_name(__file__)


def get_nodes(
    local_namespace: str,
    global_namespace: str,
    **arguments: Any,
) -> tuple[list[Node], list[ComposableNode]]:
    locomotion_controller_fqn_builder = (
        RosFQNBuilder()
        .scope(Scope.Global)
        .agent()
        .component(Component.Custom, "locomotion_controller")
    )
    locomotion_controller_name = locomotion_controller_fqn_builder.build(
        RosFQNSegment.Component
    )
    locomotion_controller_node = Node(
        package=package_name,
        executable="locomotion_controller",
        namespace=local_namespace,
        name=locomotion_controller_name,
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "config",
                    f"{locomotion_controller_name}.yaml",
                ]
            )
        ],
        remappings=[
            (
                "cmd_vel",
                locomotion_controller_fqn_builder.resource(
                    Resource.Custom, "cmd_vel"
                ).build(),
            ),
            (
                "set_state",
                locomotion_controller_fqn_builder.resource(
                    Resource.Custom, "set_state"
                ).build(),
            ),
        ],
    )

    joint_state_publisher_name = "joint_state_publisher"
    joint_state_publisher_node = Node(
        package=package_name,
        executable="joint_state_publisher",
        namespace=local_namespace,
        name=joint_state_publisher_name,
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "config",
                    f"{joint_state_publisher_name}.yaml",
                ]
            ),
            PathJoinSubstitution(
                [FindPackageShare(package_name), "config", "joint_names.yaml"]
            ),
        ],
    )

    robot_state_publisher_fqn_builder = RosFQNBuilder().scope(Scope.Global).agent()
    robot_state_publisher_name = (
        RosFQNBuilder()
        .component(Component.Custom, "robot_state_publisher")
        .build(RosFQNSegment.Component)
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=local_namespace,
        name=robot_state_publisher_name,
        output="screen",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "xacro ",
                        PathJoinSubstitution(
                            [
                                FindPackageShare("sfg_go2_description"),
                                "xacro",
                                "robot_description.urdf.xacro",
                            ]
                        ),
                    ]
                ),
                "frame_prefix": f"{robot_state_publisher_fqn_builder.build(RosFQNSegment.Agent)}/",
            }
        ],
        remappings=[
            (
                "robot_description",
                robot_state_publisher_fqn_builder.resource(
                    Resource.RobotDescription
                ).build(),
            )
        ],
    )

    return [
        locomotion_controller_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
    ], []


def generate_launch_description():
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
                name="base_platform_container",
                output="screen",
                composable_node_descriptions=composable_nodes,
            ),
        ]
    )
