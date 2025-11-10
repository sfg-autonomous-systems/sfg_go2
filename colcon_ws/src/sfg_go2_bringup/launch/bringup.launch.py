import launch
import sfg_utils.launch_utils
from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from rospkg import get_package_name
from sfg_utils.fqn import RosFqnBuilder, RosFqnSegment, Scope

package_name = get_package_name(__file__)
local_namespace = RosFqnBuilder().scope(Scope.Local).agent()
global_namespace = RosFqnBuilder().scope(Scope.Global).agent()


def generate_launch_description() -> launch.LaunchDescription:
    agent_launch_description_entities = (
        sfg_utils.launch_utils.get_launch_description_entities(
            "sfg_agent",
            "agent.launch.py",
        )
    )

    hardware_interface_launch_description_entities = (
        sfg_utils.launch_utils.get_launch_description_entities(
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

    return launch.LaunchDescription(
        [
            SetLaunchConfiguration(
                "metadata_filepath",
                PathJoinSubstitution(
                    [FindPackageShare(package_name), "config", "agent_metadata.yaml"]
                ),
            ),
            *agent_launch_description_entities.launch_arguments,
            *hardware_interface_launch_description_entities.launch_arguments,
            *agent_launch_description_entities.nodes,
            *hardware_interface_launch_description_entities.nodes,
            ComposableNodeContainer(
                package="rclcpp_components",
                executable="component_container_mt",
                namespace=local_namespace.build(
                    RosFqnSegment.Scope, RosFqnSegment.Agent
                ),
                name="bringup_container",
                output="screen",
                composable_node_descriptions=agent_launch_description_entities.composable_nodes
                + hardware_interface_launch_description_entities.composable_nodes,
            ),
            PushRosNamespace(
                local_namespace.build(RosFqnSegment.Scope, RosFqnSegment.Agent)
            ),
            lighthouse_tracker_launch_description,
        ]
    )
