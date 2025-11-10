import launch
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
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
local_namespace = RosFqnBuilder().scope(Scope.Local).agent()
global_namespace = RosFqnBuilder().scope(Scope.Global).agent()


def generate_launch_description() -> launch.LaunchDescription:
    locomotion_controller_fqn_builder = global_namespace.component(
        Component.LocomotionController
    )
    locomotion_controller_node = Node(
        package=package_name,
        executable="locomotion_controller",
        namespace=local_namespace.build(RosFqnSegment.Scope, RosFqnSegment.Agent),
        name=locomotion_controller_fqn_builder.build(RosFqnSegment.Component),
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "config",
                    f"{locomotion_controller_fqn_builder.build(RosFqnSegment.Component)}.yaml",
                ]
            )
        ],
        remappings=[
            (
                RosFqnBuilder().resource(Resource.CmdVel).build(RosFqnSegment.Resource),
                locomotion_controller_fqn_builder.resource(Resource.CmdVel).build(),
            ),
            (
                RosFqnBuilder()
                .resource(Resource.TriggerAction)
                .build(RosFqnSegment.Resource),
                locomotion_controller_fqn_builder.resource(
                    Resource.TriggerAction
                ).build(),
            ),
        ],
    )

    joint_state_publisher_name = "joint_state_publisher"
    joint_state_publisher_node = Node(
        package=package_name,
        executable="joint_state_publisher",
        namespace=local_namespace.build(RosFqnSegment.Scope, RosFqnSegment.Agent),
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

    robot_state_publisher_fqn_builder = global_namespace.component(
        Component.Custom, "robot_state_publisher"
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=local_namespace.build(RosFqnSegment.Scope, RosFqnSegment.Agent),
        name=robot_state_publisher_fqn_builder.build(RosFqnSegment.Component),
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
                RosFqnBuilder()
                .resource(Resource.RobotDescription)
                .build(RosFqnSegment.Resource),
                global_namespace.resource(Resource.RobotDescription).build(),
            )
        ],
    )

    # ToDo: I would greatly prefer running all these nodes as composable nodes in the same process
    # but since some of these nodes are using the Unitree SDK which crashes if used multiple times
    # within the same process, we resort to separate processes...
    return launch.LaunchDescription(
        [
            locomotion_controller_node,
            joint_state_publisher_node,
            robot_state_publisher_node,
        ]
    )
