# Drone Orbit & Data Collection

## Demo Video

[![Demo Video](https://img.youtube.com/vi/d-m64OR6nMQ/hqdefault.jpg)](https://www.youtube.com/watch?v=d-m64OR6nMQ)

---

## Overview

`orbit_detection.py` does 3 things simultaneously in a single terminal:

1. **Orbital flight** — drone takes off automatically, flies a circular orbit around the target (`person_1`), then lands
2. **Person detection** — real-time person detection from the drone camera using YOLOv8
3. **Dataset collection** — saves each frame together with angular velocity from the IMU (`gx, gy, gz`) for neural network training

End goal: build a dataset of **(camera frame, IMU angular velocity)** pairs — input for a neural network that learns the relationship between motion blur in images and drone rotation rate.

---

## Requirements

| Component | Version |
|---|---|
| ROS2 | Jazzy |
| PX4 SITL | main branch |
| Gazebo | Harmonic |
| Python | 3.12 (venv_ros2_yolo virtual environment) |
| CUDA | 12.x (GTX 1050 Ti or better) |

Python packages in `~/venv_ros2_yolo`:
- `ultralytics` (YOLOv8)
- `torch` (CUDA 12.1 build)
- `cv_bridge`, `rclpy` (from system ROS2)
- `opencv-python`

---

## File Structure

```
ws_drone/src/drone_sim/drone_sim/
├── orbit_detection.py      <- main file, run this
├── drone_controller.py     <- pure orbit controller + PID
├── coord_convert.py        <- ENU <-> NED coordinate conversion
└── imu_camera_collector.py <- standalone collector node (not needed separately)

~/orbit_imu_data/
└── session_YYYYMMDD_HHMMSS/
    ├── frames/
    │   ├── frame_000001.jpg
    │   ├── frame_000002.jpg
    │   └── ...
    └── imu_log.csv
```

---

## How to Run

### Terminal 1 — Start Gazebo + PX4

```bash
cd ~/DIPLOM
./run_orbit.sh
```

Wait until Gazebo opens and the drone appears in the simulation scene.

### Terminal 2 — Fly + Detect + Collect data

```bash
source /opt/ros/jazzy/setup.bash
source ~/DIPLOM/ws_drone/install/setup.bash
~/venv_ros2_yolo/bin/python \
  ~/DIPLOM/ws_drone/src/drone_sim/drone_sim/orbit_detection.py
```

The drone will automatically:
1. Wait for EKF2 ready -> enable OFFBOARD mode -> arm
2. Take off to 5m
3. Fly 2 circular laps at radius 9m around `person_1`
4. Land -> close data files -> exit

Stop early with `Ctrl+C` — data files are closed cleanly.

---

## Flight Parameters

| Parameter | Value | Description |
|---|---|---|
| Orbit center | ENU(10, 0, 0) | Position of `person_1` in Gazebo |
| Radius | 9 m | Orbit radius |
| Altitude | 5 m | Flight altitude |
| Angular speed | 0.4 rad/s | Approx. 16 seconds per lap |
| Laps | 2 | Total number of laps |
| Max horizontal speed | 4 m/s | Horizontal velocity limit |

---

## Output Data

File `imu_log.csv`:

| Column | Description |
|---|---|
| `frame_id` | Frame sequence number |
| `cam_ts_us` | Frame capture time from Gazebo simulation clock (seconds) |
| `imu_ts_us` | IMU sample time from PX4 internal clock (µs) |
| `dt_sync_ms` | Time delta between frame and IMU sample (ms) |
| `gx_rad_s` | Angular velocity around X axis (rad/s) |
| `gy_rad_s` | Angular velocity around Y axis (rad/s) |
| `gz_rad_s` | Angular velocity around Z axis — yaw rate (rad/s) |
| `ax_m_s2` | Acceleration X axis (m/s²) |
| `ay_m_s2` | Acceleration Y axis (m/s²) |
| `az_m_s2` | Acceleration Z axis (m/s²) |

Each CSV row corresponds to one `frame_XXXXXX.jpg` file in the `frames/` directory.

---

## Controller Architecture

```
Pure Pursuit Orbit Controller
  └── compute next target point on circle
        └── velocity = (target - current position) x gain
              └── clamped -> send velocity setpoint to PX4
```

Reason for using **velocity setpoints** instead of position setpoints: avoids PX4 accelerating hard then braking sharply, which causes the drone to wobble.

---

## Technical Notes

- Camera is tilted 30° (0.524 rad) — looking down and forward
- Detection model: `yolov8s.pt`, confidence threshold 0.7, persons only
- IMU subscribed with `BEST_EFFORT + TRANSIENT_LOCAL` QoS to match PX4 publisher
- Simulation dataset used for initial training, then fine-tuned on real data
