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


def get_nodes(**arguments: Any) -> tuple[list[Node], list[ComposableNode]]:
    locomotion_controller_fqn_builder = (
        RosFqnBuilder()
        .scope(Scope.Global)
        .agent()
        .component(Component.Custom, "locomotion_controller")
    )
    locomotion_controller_name = locomotion_controller_fqn_builder.build(
        RosFqnSegment.Component
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
                locomotion_controller_fqn_builder.resource(Resource.CmdVel).build(
                    RosFqnSegment.Resource
                ),
                locomotion_controller_fqn_builder.build(),
            ),
            (
                locomotion_controller_fqn_builder.resource(
                    Resource.Custom, "set_state"
                ).build(RosFqnSegment.Resource),
                locomotion_controller_fqn_builder.build(),
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

    robot_state_publisher_fqn_builder = RosFqnBuilder().scope(Scope.Global).agent()
    robot_state_publisher_name = (
        RosFqnBuilder()
        .component(Component.Custom, "robot_state_publisher")
        .build(RosFqnSegment.Component)
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
                "frame_prefix": f"{robot_state_publisher_fqn_builder.build(RosFqnSegment.Agent)}/",
            }
        ],
        remappings=[
            (
                robot_state_publisher_fqn_builder.resource(
                    Resource.RobotDescription
                ).build(RosFqnSegment.Resource),
                robot_state_publisher_fqn_builder.build(),
            )
        ],
    )

    # ToDo: I would greatly prefer running all these nodes as composable nodes in the same process
    # but since some of these nodes are using the Unitree SDK which crashes if used multiple times
    # within the same process, we resort to separate processes...
    return [
        locomotion_controller_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
    ], []


def generate_launch_description() -> launch.LaunchDescription:
    nodes, composable_nodes = get_nodes()

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
