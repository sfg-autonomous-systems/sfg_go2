import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
                    "camera_namespace": "go2",
                    "camera_name": "camera_head",
                    "serial_no": "'728312070152'",
                    "rgb_camera.color_profile": "1280x720x30",
                    "depth_module.depth_profile": "1280x720x30",
                    "align_depth.enable": "true",
                    "enable_sync": "true",
                    "accelerate_gpu_with_glsl": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("livox_ros_driver2"),
                        "launch_ROS2",
                        "livox_ros_driver2_launch.py",
                    ),
                ),
                launch_arguments={
                    "xfer_format": "0",
                    "user_config_path": os.path.join(
                        get_package_share_directory("go2"),
                        "config",
                        "livox_mid360_config.json",
                    ),
                }.items(),
            ),
        ]
    )
