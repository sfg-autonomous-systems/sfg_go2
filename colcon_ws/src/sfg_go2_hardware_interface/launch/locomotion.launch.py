import launch
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from rospkg import get_package_name
from sfg_utils import get_ros_namespaces

package_name = get_package_name(__file__)
local_namespace, global_namespace = get_ros_namespaces()


def generate_launch_description():
    # ToDo: Right now we cannot start multiple unitree_sdk2 nodes in the same process
    # presumably because they both try to initialize their singleton CycloneDDS-based ChannelFactory.
    # Perhaps we can fix this somehow?
    locomotion_controller_node = Node(
        package=package_name,
        executable="locomotion_controller",
        namespace=local_namespace,
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
                global_namespace + "/locomotion_controller/cmd_vel",
            ),
            (
                "locomotion_controller/set_locomotion_mode",
                global_namespace + "/locomotion_controller/set_locomotion_mode",
            ),
        ],
    )

    container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_mt",
        namespace=local_namespace,
        name="container",
        output="screen",
        composable_node_descriptions=(
            ComposableNode(
                package=package_name,
                plugin=f"{package_name}::JointStatePublisher",
                namespace=local_namespace,
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
            ),
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                namespace=local_namespace,
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
            ),
        ),
    )

    return launch.LaunchDescription([locomotion_controller_node, container])
