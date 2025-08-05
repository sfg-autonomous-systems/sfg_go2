import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from rospkg import get_package_name

package_name = get_package_name(__file__)


def generate_launch_description():
    agent_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("sfg_agent"),
                        "launch",
                        "agent.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "metadata_filepath": PathJoinSubstitution(
                [FindPackageShare(package_name), "config", "agent_metadata.yaml"]
            )
        }.items(),
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

    hardware_interface_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("sfg_go2_hardware_interface"),
                        "launch",
                        "hardware_interface.launch.py",
                    ]
                )
            ]
        ),
    )

    auxiliary_sensors_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare(package_name),
                        "launch",
                        "payload_sensors.launch.py",
                    ]
                )
            ]
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
            agent_launch_description,
            jtop_launch_description,
            auxiliary_sensors_launch_description,
            hardware_interface_launch_description,
            lighthouse_tracker_launch_description,
        ]
    )
