# PX4 Drone Control with ROS2

Guide for autonomously controlling a PX4 drone using ROS2.

## Setup (Run Once)

### 1. Install PX4-ROS2 Bridge

```bash
cd /home/ryan-le-ai/DIPLOM/ws_drone
./setup_px4_ros2.sh
```

This script will clone and build:
- `px4_msgs`: ROS2 message definitions for PX4
- `px4_ros_com`: Bridge between PX4 and ROS2

### 2. Build workspace

```bash
cd /home/ryan-le-ai/DIPLOM/ws_drone
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### 3. Install MicroXRCEAgent (if not already installed)

```bash
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

---

## Usage

### Method 1: Automatic (Launch file)

Open 3 terminals:

**Terminal 1: PX4 SITL**
```bash
cd ~/PX4-Autopilot
PX4_GZ_MODEL_POSE="0.002,733.17,2,0,0,0" \
PX4_GZ_MODEL=x500_mono_cam \
PX4_GZ_WORLD=drone_neighborhood \
make px4_sitl gz_x500_mono_cam
```

**Terminal 2: MicroXRCEAgent**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 3: ROS2 Launch**
```bash
cd /home/ryan-le-ai/DIPLOM/ws_drone
source install/setup.bash
ros2 launch drone_sim px4_drone_control.launch.py
```

The drone will automatically:
1. Arm
2. Switch to OFFBOARD mode
3. Take off to 5m
4. Hold position

---

### Method 2: Manual (Step by step)

**Terminal 1: Gazebo**
```bash
cd /home/ryan-le-ai/DIPLOM/ws_drone
source install/setup.bash
export GZ_SIM_RESOURCE_PATH="$(pwd)/install/drone_sim/share/drone_sim/models:~/PX4-Autopilot/Tools/simulation/gz/models"
gz sim -r config/drone_neighborhood.sdf
```

**Terminal 2: PX4 SITL**
```bash
cd ~/PX4-Autopilot
PX4_GZ_MODEL_POSE="0.002,733.17,2,0,0,0" \
PX4_GZ_MODEL=x500_mono_cam \
PX4_GZ_WORLD=drone_neighborhood \
make px4_sitl gz_x500_mono_cam
```

**Terminal 3: MicroXRCEAgent**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 4: ROS2 Controller**
```bash
cd /home/ryan-le-ai/DIPLOM/ws_drone
source install/setup.bash
ros2 run drone_sim px4_takeoff_controller
```

---

## Checking ROS2 Topics

List all PX4 topics:
```bash
ros2 topic list | grep fmu
```

View drone status:
```bash
ros2 topic echo /fmu/out/vehicle_status
```

View drone position:
```bash
ros2 topic echo /fmu/out/vehicle_local_position
```

---

## Changing Takeoff Altitude

### Option 1: Launch parameter
```bash
ros2 launch drone_sim px4_drone_control.launch.py takeoff_altitude:=10.0
```

### Option 2: Edit code
Open `/home/ryan-le-ai/DIPLOM/ws_drone/src/drone_sim/drone_sim/px4_takeoff_controller.py`

Find the line:
```python
self.takeoff_altitude = 5.0  # meters
```

Change it to:
```python
self.takeoff_altitude = 10.0  # meters
```

Rebuild:
```bash
cd /home/ryan-le-ai/DIPLOM/ws_drone
source /opt/ros/jazzy/setup.bash
colcon build --packages-select drone_sim
source install/setup.bash
```

---

## Troubleshooting

### 1. "No topics from /fmu"
→ MicroXRCEAgent is not running or PX4 has not connected yet.

### 2. "Drone will not arm"
→ Check the PX4 console for errors (GPS, safety switch, etc.)

### 3. "Drone will not fly"
→ Check whether offboard mode engaged:
```bash
ros2 topic echo /fmu/out/vehicle_status --field nav_state
```
Nav state 14 = OFFBOARD mode

### 4. "Import px4_msgs error"
→ Workspace not sourced or px4_msgs not built:
```bash
cd /home/ryan-le-ai/DIPLOM/ws_drone
source install/setup.bash
```

---

## Extension: Object Tracking

To make the drone follow an object, you need:

1. **Computer Vision node**: Detect objects from the camera
   - Subscribe: `/camera` (sensor_msgs/Image)
   - Publish: `/target_position` (geometry_msgs/Point)

2. **Tracking controller**: Control the drone toward the target
   - Subscribe: `/target_position`
   - Publish: `/fmu/in/trajectory_setpoint`

3. **YOLO detection node** (to be created):
   - Load YOLO model
   - Detect objects from camera
   - Calculate 3D position from image coordinates

---

## Architecture

```
┌─────────────┐
│   Gazebo    │ (Simulation)
│ + x500 drone│
└──────┬──────┘
       │ Topics: /camera, /imu, /gps...
       ↓
┌──────────────┐
│  PX4 SITL    │ (Autopilot firmware)
└──────┬───────┘
       │ MAVLINK over UDP
       ↓
┌──────────────────┐
│ MicroXRCEAgent   │ (DDS Bridge)
└──────┬───────────┘
       │ ROS2 DDS
       ↓
┌─────────────────────────────────┐
│ ROS2 Nodes                      │
│ - px4_takeoff_controller        │
│ - object_detector (YOLO)        │
│ - tracking_controller           │
└─────────────────────────────────┘
```

---

## Files

- `px4_takeoff_controller.py`: Node for controlling drone takeoff
- `px4_drone_control.launch.py`: Combined launch file
- `setup_px4_ros2.sh`: Script to install PX4-ROS2 bridge
- `README_PX4_CONTROL.md`: This file

---

## Next Steps

1. ✅ Setup PX4-ROS2 communication
2. ✅ Implement takeoff controller
3. ⬜ Implement YOLO object detection
4. ⬜ Implement object tracking controller
5. ⬜ Collect training data for YOLO
