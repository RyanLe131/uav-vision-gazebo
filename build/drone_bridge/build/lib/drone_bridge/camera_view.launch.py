import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("drone_bridge")
    gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "gz_bridge.launch.py")
        )
    )

    viewer = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="rqt_image_view",
        arguments=["/camera/image"],
        output="screen",
    )

    return LaunchDescription([gz_bridge, viewer])
