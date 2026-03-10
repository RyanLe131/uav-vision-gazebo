from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    camera_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='camera_view',
        remappings=[
            ('image', '/world/iris_runway/model/iris_with_gimbal/model/gimbal/'
             'link/pitch_link/sensor/camera/image')
        ],
        output='screen'
    )

    return LaunchDescription([
        camera_view,
    ])
