import launch
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from rospkg import get_package_name
from sfg_utils import get_ros_namespaces

package_name = get_package_name(__file__)
local_namespace, global_namespace = get_ros_namespaces()


def generate_launch_description():
    locomotion_controller = Node(
        package=package_name,
        executable="locomotion_controller",
        namespace=local_namespace,
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare(package_name), "config", "locomotion_controller.yaml"]
            )
        ],
        remappings=[
            (
                "locomotion_controller/cmd_vel",
                global_namespace + "/locomotion_controller/cmd_vel",
            ),
            (
                "locomotion_controller/set_locomotion_mode",
                global_namespace + "/locomotion_controller/set_locomotion_mode",
            ),
        ],
    )

    return launch.LaunchDescription([locomotion_controller])
