import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_share = get_package_share_directory("drone_bridge")
    gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "gz_bridge.launch.py")
        )
    )

    note = LogInfo(
        msg="OpenCV viewer not configured. Use camera_view.launch.py for rqt_image_view."
    )

    return LaunchDescription([gz_bridge, note])
