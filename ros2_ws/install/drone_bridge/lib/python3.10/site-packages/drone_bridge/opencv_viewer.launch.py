from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    camera_viewer = Node(
        package='drone_bridge',
        executable='camera_viewer.py',
        name='camera_viewer_opencv',
        output='screen'
    )

    return LaunchDescription([
        camera_viewer,
    ])
