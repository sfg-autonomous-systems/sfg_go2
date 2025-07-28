from pathlib import Path

import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from sfg_utils import get_agent_name, sanitize_agent_name

package_directory = Path(get_package_share_directory("sfg_go2_locomotion"))
sanitized_agent_name = sanitize_agent_name(get_agent_name())
local_namespace = "/local"
global_namespace = "/global/" + sanitized_agent_name


def generate_launch_description():
    locomotion_controller = Node(
        package="sfg_go2_locomotion",
        executable="locomotion_controller",
        namespace=local_namespace,
        name="locomotion_controller",
        parameters=[
            package_directory / "config" / "locomotion_controller.yaml",
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
