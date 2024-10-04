from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace

package_name = "go2"


def generate_launch_description():
    return LaunchDescription(
        [
            PushRosNamespace("go2"),
            Node(
                package=package_name,
                executable="camera",
            ),
        ]
    )
