# How to Run the Simulation — DIPLOM Drone

---

## Environment Requirements

| Component | Version |
|-----------|---------|
| ROS2 | Jazzy |
| Python | 3.12 (system, do not use conda/venv) |
| PX4-Autopilot | built at `~/PX4-Autopilot` |
| MicroXRCEAgent | installed at `/usr/local/bin/MicroXRCEAgent` |
| Gazebo | Harmonic (gz sim) |

---

## Step 0 — Check Environment (REQUIRED before running)

```bash
source /opt/ros/jazzy/setup.bash
source /home/ryan-le-ai/DIPLOM/ws_drone/install/setup.bash
bash /home/ryan-le-ai/DIPLOM/ws_drone/check_env.sh
```

You must see `ALL OK` before proceeding.

> **Note:** No conda env or venv should be active when running ROS2.
> If using conda: `conda deactivate`

---

## Step 1 — Build (only needed once or after code changes)

```bash
source /opt/ros/jazzy/setup.bash
cd /home/ryan-le-ai/DIPLOM/ws_drone

# Build everything
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3.12

# Or build only drone_sim (when editing Python files only)
colcon build --packages-select drone_sim

source install/setup.bash
```

---

## Step 2 — Run the Simulation

### Terminal 1 — Launch Gazebo + PX4 + Bridge

```bash
source /opt/ros/jazzy/setup.bash
source /home/ryan-le-ai/DIPLOM/ws_drone/install/setup.bash
ros2 launch drone_sim baylands_px4.launch.py
```

The launch file starts processes in order:
- `t=0s`  — Gazebo server + GUI, loads world `baylands`
- `t=8s`  — PX4 SITL spawns drone `x500_mono_cam`
- `t=20s` — Camera bridge `/camera/image_raw` + clock bridge

Wait until you see: `[px4] INFO  Ready to fly`

---

### Terminal 2 — Orbit Data Collector

```bash
bash ~/DIPLOM/run_orbit.sh           # run only (already built)
bash ~/DIPLOM/run_orbit.sh --build   # build first then run
```

The drone will automatically:
1. Wait for EKF2 ready
2. Switch to OFFBOARD + ARM
3. Fly to orbit start point
4. Fly `LAPS` circular laps around `person_1`, recording video
5. Land automatically

---

## Customizing Parameters

All parameters are in the `Constants` section of:
`ws_drone/src/drone_sim/drone_sim/orbit_data_collector.py`

```python
TARGET_X       =  0.0    # NED coordinate of person_1 (auto from coord_convert)
TARGET_Y       = 10.0    # NED coordinate of person_1 (auto from coord_convert)
ORBIT_RADIUS   =  5.0    # orbit radius (m)
ORBIT_ALTITUDE =  4.0    # flight altitude (m)
ORBIT_SPEED    =  0.3    # angular speed (rad/s) -> linear speed = RADIUS x SPEED
LAPS           =  3      # number of laps
MAX_RECORD_SEC = 300.0   # video recording limit: 5 minutes
VIDEO_NAME     = 'orbit_test'   # <- CHANGE VIDEO NAME HERE
```

> After changing constants -> **rebuild required**: `bash ~/DIPLOM/run_orbit.sh --build`

---

## Flight Speed

Actual linear speed = `ORBIT_RADIUS x ORBIT_SPEED`

| ORBIT_SPEED | Linear speed | Notes |
|-------------|--------------|-------|
| 0.2 rad/s | 1.0 m/s | Slow, stable |
| 0.3 rad/s | 1.5 m/s | Default |
| 0.5 rad/s | 2.5 m/s | Fast, drone may lag |
| 1.0 rad/s | 5.0 m/s | Maximum recommended limit |

---

## Coordinate Systems

### Gazebo SDF uses ENU
```
x = East
y = North
z = Up

<pose> x  y  z  roll  pitch  yaw </pose>
  roll  = rotation around X axis (East)
  pitch = rotation around Y axis (North)  <- used for camera angle
  yaw   = rotation around Z axis (Up)
```

### PX4 / Python code uses NED
```
x = North
y = East
z = Down  (negative = above ground)
```

### Conversion
Use `coord_convert.py` — **never copy coordinates directly from SDF into code**:
```python
from drone_sim.coord_convert import enu_to_ned
ned = enu_to_ned(sdf_x, sdf_y, sdf_z)
```

---

## Character Positions in the Baylands World

| Name | SDF ENU (x, y, z) | NED PX4 (x, y) |
|------|-------------------|----------------|
| person_1 (Male visitor) | 10, 0, 1.0 | 0.0, 10.0 |
| person_2 (Nurse Female) | 15, 0, 0.0 | 0.0, 15.0 |
| person_3 (Patient Walking Cane) | 20, 0, 0.0 | 0.0, 20.0 |

---

## Camera

| Parameter | Value |
|-----------|-------|
| Model | `x500_mono_cam` (fixed mount, no gimbal) |
| Config file | `~/PX4-Autopilot/Tools/simulation/gz/models/x500_mono_cam/model.sdf` |
| Position on drone | x=0.12m (front), y=0.03m (right), z=0.242m (top) |
| Current pitch angle | **1.047 rad = 60°** looking down from horizontal |
| Horizontal FOV | 1.74 rad ≈ 99.7° |
| Vertical FOV | ~80.8° |
| Resolution | 1280 x 960 |
| Frame rate | 30 Hz |
| ROS2 topic | `/camera/image_raw` |

### Camera Pitch Angle Explained

```
pitch = 0        -> looking straight forward (parallel to ground)
pitch = 1.047    -> tilted 60° downward from horizontal  <- current
pitch = 1.5707   -> looking straight down 90° (nadir)
```

`pitch` in SDF is **rotation around the Y axis (North)** by the right-hand rule:
- Positive value = camera nose pointing down toward ground
- Negative value = camera nose pointing up toward sky

To change the camera angle, edit the `<pose>` line in `x500_mono_cam/model.sdf`:
```xml
<pose>.12 .03 .242 0 [PITCH_RAD] 0</pose>
```
No rebuild needed — just restart Gazebo.

---

## Recorded Video

- Save location: `~/orbit_videos/`
- Filename: `{VIDEO_NAME}_{YYYYMMDD_HHMMSS}.mp4`
- Recording starts: when drone enters **ORBIT** state
- Recording stops: after `LAPS` laps **or** after `MAX_RECORD_SEC` seconds
- Overlay: state, lap, angle, elapsed time, NED coordinates

Change video name before running:
```python
# orbit_data_collector.py
VIDEO_NAME = 'your_name_here'
```

---

## Overall Timeline

```
t=0s    Terminal 1: ros2 launch -> Gazebo + PX4 start
t=8s    PX4 SITL spawns drone at (0,0,0)
t=20s   Camera bridge starts
t=30s   Terminal 2: run_orbit.sh -> node starts
t=~50s  Drone OFFBOARD + ARM
t=~60s  TAKEOFF -> fly to orbit start point (5, 10) NED
t=~90s  ORBIT starts -> video recording begins
t=~3m   Completes LAPS laps -> LAND -> video saved
```

---

## Common Errors

### `No module named 'px4_msgs'`
```bash
source /opt/ros/jazzy/setup.bash
source /home/ryan-le-ai/DIPLOM/ws_drone/install/setup.bash
```

### `libpython3.13.so: cannot open shared object file`
```bash
cd /home/ryan-le-ai/DIPLOM/ws_drone
rm -rf build/px4_msgs install/px4_msgs
rm -rf build/px4_ros_com install/px4_ros_com
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_msgs px4_ros_com \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3.12
source install/setup.bash
```

### Drone does not arm
- Check that MicroXRCEAgent is running
- Wait an additional 10-20s

### Topic `/camera/image_raw` not found
```bash
ros2 topic list | grep camera
```
If missing -> bridge not yet running, wait for PX4 to finish spawning drone (~20s).

### Drone flies to wrong position / orbit center is off
- Check the `[DRONE SPAWN]` log — drone must spawn at `(0, 0, 0)` NED
- If wrong -> verify `PX4_GZ_MODEL_POSE="0,0,0,0,0,0"` is set in the launch file
- Check the `[ORBIT START]` log — distance to person_1 should be approximately 5.0m

---

## Related File Structure

```
DIPLOM/
├── run_orbit.sh                          <- Quick run script (--build to build first)
└── ws_drone/
    └── src/drone_sim/
        ├── config/baylands.sdf           <- Gazebo world (character positions, terrain)
        ├── drone_sim/
        │   ├── coord_convert.py          <- ENU <-> NED conversion
        │   ├── orbit_data_collector.py   <- Main node: orbit + video recording
        │   ├── image_collector.py        <- Live camera viewer
        │   └── px4_takeoff_controller.py <- Simple takeoff node
        └── launch/
            └── baylands_px4.launch.py    <- Main launch file

~/PX4-Autopilot/Tools/simulation/gz/models/
    └── x500_mono_cam/model.sdf           <- Camera angle configuration
```
