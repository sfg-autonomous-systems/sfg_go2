import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    realsense_launch_path = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py"
    )

    return launch.LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(realsense_launch_path),
                launch_arguments={
                    "camera_namespace": "go2",
                    "camera_name": "camera_head",
                    "serial_no": "'728312070152'",
                    "rgb_camera.color_profile": "1280x720x15",
                    "depth_module.depth_profile": "1280x720x15",
                    "align_depth.enable": "true",
                    "enable_sync": "true",
                    "accelerate_gpu_with_glsl": "true",
                }.items(),
            )
        ]
    )
