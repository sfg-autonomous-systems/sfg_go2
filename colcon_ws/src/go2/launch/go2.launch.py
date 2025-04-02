import os
import socket

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

namespace = socket.gethostname().replace("-", "_")


def generate_launch_description():
    return launch.LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("realsense2_camera"),
                        "launch",
                        "rs_launch.py",
                    ),
                ),
                launch_arguments={
                    "camera_namespace": namespace,
                    "camera_name": "camera_head",
                    "serial_no": "'728312070152'",
                    "rgb_camera.color_profile": "1280x720x30",
                    "depth_module.depth_profile": "1280x720x30",
                    "align_depth.enable": "true",
                    "enable_sync": "true",
                    "accelerate_gpu_with_glsl": "true",
                }.items(),
            ),
        ]
    )
