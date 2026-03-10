from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo


def generate_launch_description():
    """
    Launch the Gazebo-ROS bridge for camera and IMU sensors.
    This bridge connects Gazebo topics to ROS2 topics.
    
    Bridges:
    - Camera: /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
    - IMU: /world/iris_runway/model/iris_with_gimbal/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu
    """
    
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/'
            'pitch_link/sensor/camera/image@sensor_msgs/msg/Image@'
            'gz.msgs.Image'
        ],
        parameters=[{'lazy': False}],
        output='screen',
        name='camera_bridge'
    )

    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/iris_runway/model/iris_with_gimbal/model/iris_with_'
            'standoffs/link/imu_link/sensor/imu_sensor/imu@'
            'sensor_msgs/msg/Imu@gz.msgs.IMU'
        ],
        parameters=[{'lazy': False}],
        output='screen',
        name='imu_bridge'
    )

    log_msg = LogInfo(msg="Gazebo-ROS Bridge launched. Camera and IMU topics are now available in ROS2.")

    return LaunchDescription([
        log_msg,
        camera_bridge,
        imu_bridge,
    ])
