import launch
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
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
local_namespace, global_namespace = (
    RosFQNBuilder().scope(Scope.Local).agent().build(end=RosFQNSegment.Agent),
    RosFQNBuilder().scope(Scope.Global).agent().build(end=RosFQNSegment.Agent),
)


def generate_launch_description():
    locomotion_controller_fqn_builder = (
        RosFQNBuilder()
        .scope(Scope.Global)
        .agent()
        .component(Component.Custom, "locomotion_controller")
    )

    locomotion_controller_node = Node(
        package=package_name,
        executable="locomotion_controller",
        namespace=local_namespace,
        name="locomotion_controller",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "config",
                    "locomotion_controller.yaml",
                ]
            )
        ],
        remappings=[
            (
                "locomotion_controller/cmd_vel",
                locomotion_controller_fqn_builder.resource(
                    Resource.Custom, "cmd_vel"
                ).build(),
            ),
            (
                "locomotion_controller/set_state",
                locomotion_controller_fqn_builder.resource(
                    Resource.Custom, "set_state"
                ).build(),
            ),
        ],
    )

    joint_state_publisher_node = Node(
        package=package_name,
        executable="joint_state_publisher",
        namespace=local_namespace,
        name="joint_state_publisher",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "config",
                    "joint_state_publisher.yaml",
                ]
            ),
            PathJoinSubstitution(
                [FindPackageShare(package_name), "config", "joint_names.yaml"]
            ),
        ],
    )

    robot_state_publisher_fqn_builder = RosFQNBuilder().scope(Scope.Global).agent()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=local_namespace,
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "cat ",
                        PathJoinSubstitution(
                            [
                                FindPackageShare("sfg_go2_description"),
                                "urdf",
                                "robot_description.urdf",
                            ]
                        ),
                    ]
                )
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

    return launch.LaunchDescription(
        [
            locomotion_controller_node,
            joint_state_publisher_node,
            robot_state_publisher_node,
        ]
    )
