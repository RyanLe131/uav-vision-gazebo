from launch import LaunchDescription
from launch_ros.actions import Node

GZ_IMAGE_TOPIC = (
    "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"
)
GZ_CAMERA_INFO_TOPIC = (
    "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/camera_info"
)
GZ_IMU_TOPIC = (
    "/world/iris_runway/model/iris_with_gimbal/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu"
)


def generate_launch_description():
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_camera_bridge",
        parameters=[
            {"use_sim_time": True},
            {"bridge_names": ["camera_bridge", "camera_info_bridge", "imu_bridge"]},
            {"bridges.camera_bridge.ros_topic_name": "/camera/image"},
            {"bridges.camera_bridge.gz_topic_name": GZ_IMAGE_TOPIC},
            {"bridges.camera_bridge.ros_type_name": "sensor_msgs/msg/Image"},
            {"bridges.camera_bridge.gz_type_name": "gz.msgs.Image"},
            {"bridges.camera_bridge.direction": "GZ_TO_ROS"},
            {"bridges.camera_bridge.qos_profile": "SENSOR_DATA"},
            {"bridges.camera_info_bridge.ros_topic_name": "/camera/camera_info"},
            {"bridges.camera_info_bridge.gz_topic_name": GZ_CAMERA_INFO_TOPIC},
            {"bridges.camera_info_bridge.ros_type_name": "sensor_msgs/msg/CameraInfo"},
            {"bridges.camera_info_bridge.gz_type_name": "gz.msgs.CameraInfo"},
            {"bridges.camera_info_bridge.direction": "GZ_TO_ROS"},
            {"bridges.camera_info_bridge.qos_profile": "SENSOR_DATA"},
            {"bridges.imu_bridge.ros_topic_name": "/imu"},
            {"bridges.imu_bridge.gz_topic_name": GZ_IMU_TOPIC},
            {"bridges.imu_bridge.ros_type_name": "sensor_msgs/msg/Imu"},
            {"bridges.imu_bridge.gz_type_name": "gz.msgs.IMU"},
            {"bridges.imu_bridge.direction": "GZ_TO_ROS"},
            {"bridges.imu_bridge.qos_profile": "SENSOR_DATA"},
        ],
        output="screen",
    )

    return LaunchDescription([bridge])
