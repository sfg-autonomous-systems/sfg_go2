import launch
from launch.substitutions import PathJoinSubstitution
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
    trajectory_controller_fqn_builder = global_namespace.component(
        Component.Custom, "trajectory_controller"
    )
    locomotion_controller_fqn_builder = global_namespace.component(
        Component.LocomotionController
    )
    return launch.LaunchDescription(
        [
            Node(
                package=package_name,
                executable="trajectory_controller",
                namespace=local_namespace.build(
                    RosFqnSegment.Scope, RosFqnSegment.Agent
                ),
                name=trajectory_controller_fqn_builder.build(RosFqnSegment.Component),
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare(package_name),
                            "config",
                            f"{trajectory_controller_fqn_builder.build(RosFqnSegment.Component)}.yaml",
                        ]
                    )
                ],
                remappings=[
                    (
                        "cmd_vel",
                        locomotion_controller_fqn_builder.resource(
                            Resource.CmdVel
                        ).build(),
                    )
                ],
            )
        ]
    )
