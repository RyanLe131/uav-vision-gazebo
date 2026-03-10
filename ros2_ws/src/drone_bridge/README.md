# Drone Bridge

## Camera troubleshooting (ROS 2 + Gazebo)

### Environment

- ROS 2: Humble
- Gazebo: Fortress (gz-sim)
- Model: `iris_with_gimbal` with `gimbal_small_3d` camera sensor
- Expected ROS image topic: `/camera/image`

### Current issue (what is wrong)

The camera stream appears in Gazebo (`gz topic -e` shows frames), but ROS shows a publisher on `/camera/image` with no messages. For example, `ros2 topic hz /camera/image` stalls with no output, and `ros2 topic echo --once /camera/image` returns nothing.

### What works (confirmed)

- Gazebo publishes frames on:
  `/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image`
- `gz topic -e` prints `gz.msgs.Image` frames continuously.
- A ROS publisher appears on `/camera/image` after starting the bridge.

### What does not work (still broken)

- ROS tools do not receive any image messages on `/camera/image`.
- `ros2 topic hz /camera/image` never prints a rate.
- `rqt_image_view` shows no image.

### Steps tried (not resolved yet)

1) Enabled streaming for the Gazebo camera sensor:

```bash
gz topic -t /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image/enable_streaming -m gz.msgs.Boolean -p "data: 1"
```

2) Launched the bridge and viewer (bridge + `rqt_image_view`):

```bash
ros2 launch drone_bridge camera_view.launch.py
```

3) Bridged the camera topic directly:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image
```

4) Forced BEST_EFFORT QoS and remapped to `/camera/image`:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args \
  -r /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image:=/camera/image \
  -p qos_overrides./camera/image.publisher.reliability:=best_effort \
  -p qos_overrides./camera/image.publisher.history:=keep_last \
  -p qos_overrides./camera/image.publisher.depth:=5
```

### Evidence collected

- `gz topic -e` confirms the Gazebo camera sensor is publishing.
- `ros2 topic info -v /camera/image` shows:
  - Publisher is `ros_gz_bridge`
  - Reliability is `BEST_EFFORT`
  - There is at least one ROS subscriber (`ros2cli` / `rqt_image_view`)
- Despite this, no ROS messages arrive on `/camera/image`.

### Why it is not fixed yet

The Gazebo side is confirmed to be publishing, and the ROS bridge is visible with matching QoS, yet no ROS messages arrive. This indicates a deeper bridge issue that still needs investigation, for example:

- A bridge configuration detail not exposed in the launch file (e.g., incorrect direction or a hidden lazy setting).
- A message conversion or transport issue inside `ros_gz_bridge` for this sensor stream.
- A mismatch between the `gz` transport used by the sensor and what `ros_gz_bridge` is subscribing to.

At this stage, the issue is reproducible and documented, but a root-cause fix has not been confirmed.
