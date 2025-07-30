import launch
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from rospkg import get_package_name
from sfg_utils import get_ros_namespaces

package_name = get_package_name(__file__)
local_namespace, global_namespace = get_ros_namespaces()


def generate_launch_description():
    joint_state_publisher = Node(
        package=package_name,
        executable="joint_state_publisher",
        namespace=local_namespace,
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare(package_name), "config", "joint_state_publisher.yaml"]
            ),
            PathJoinSubstitution(
                [FindPackageShare(package_name), "config", "joint_names.yaml"]
            ),
        ],
    )

    return launch.LaunchDescription([joint_state_publisher])
