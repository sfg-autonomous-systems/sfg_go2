import launch
import sfg_utils.launch_utils
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from rospkg import get_package_name
from sfg_utils.fqn import RosFqnBuilder, RosFqnSegment, Scope

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
    agent_nodes, agent_composable_nodes = sfg_utils.launch_utils.get_nodes(
        "sfg_agent",
        "agent.launch.py",
        metadata_filepath=PathJoinSubstitution(
            [FindPackageShare(package_name), "config", "agent_metadata.yaml"]
        ),
    )

    hardware_interface_nodes, hardware_interface_composable_nodes = (
        sfg_utils.launch_utils.get_nodes(
            "sfg_go2_hardware_interface",
            "hardware_interface.launch.py",
        )
    )

    lighthouse_tracker_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("sfg_lighthouse_tracking"),
                        "launch",
                        "lighthouse_tracker.launch.py",
                    ]
                )
            ]
        )
    )

    jtop_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("isaac_ros_jetson_stats"),
                        "launch",
                        "jtop.launch.py",
                    ]
                )
            ]
        ),
    )

    return launch.LaunchDescription(
        [
            *agent_nodes,
            *hardware_interface_nodes,
            ComposableNodeContainer(
                package="rclcpp_components",
                executable="component_container_mt",
                namespace=local_namespace,
                name="bringup_container",
                output="screen",
                composable_node_descriptions=agent_composable_nodes
                + hardware_interface_composable_nodes,
            ),
            lighthouse_tracker_launch_description,
            jtop_launch_description,
        ]
    )
