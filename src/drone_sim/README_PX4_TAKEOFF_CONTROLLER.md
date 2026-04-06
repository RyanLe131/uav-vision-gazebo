# PX4 Takeoff Controller — Functional Documentation

File: `drone_sim/px4_takeoff_controller.py`

---

## Overview

A ROS2 node that automatically arms a PX4 drone, switches it to OFFBOARD mode, takes off to a preset altitude, and then holds position.

### Topics

| Direction | Topic | Purpose |
|-----------|-------|---------|
| Publish | `/fmu/in/offboard_control_mode` | Declare the control mode type |
| Publish | `/fmu/in/trajectory_setpoint` | Send target coordinates |
| Publish | `/fmu/in/vehicle_command` | Send MAVLink commands |
| Subscribe | `/fmu/out/vehicle_status` | Receive drone state |

---

## Class `PX4TakeoffController`

### `__init__(self)`
Initializes the node. Sets up:
- QoS profile compatible with PX4 (Best Effort, Transient Local)
- 3 publishers and 1 subscriber as listed above
- State variables: `nav_state`, `arming_state`, `offboard_setpoint_counter`
- Default takeoff altitude: `takeoff_altitude = 5.0` m
- Control loop timer running at **50 Hz** (every 0.02 seconds)

---

### `vehicle_status_callback(self, msg)`
Callback that receives messages from `/fmu/out/vehicle_status`.
Updates two state variables:
- `nav_state` — current flight mode (e.g. OFFBOARD, MANUAL, ...)
- `arming_state` — current arm/disarm state of the drone

---

### `control_loop(self)`
Main loop, called 50 times per second by the timer.
Sequence of operations each cycle:
1. Call `publish_offboard_control_mode()` — continuously declare position control mode
2. Call `publish_trajectory_setpoint()` — continuously send target coordinates
3. After **10 cycles** (~0.2s): call `engage_offboard_mode()` to request OFFBOARD mode
4. After **20 cycles** (~0.4s): call `arm()` to arm the drone
5. Counter `offboard_setpoint_counter` increments up to 100 then stops

> Reason for sending setpoints before arming: PX4 requires receiving at least a few consecutive setpoints before accepting a switch into OFFBOARD mode.

---

### `publish_offboard_control_mode(self)`
Publishes an `OffboardControlMode` message to `/fmu/in/offboard_control_mode`.
Declares to PX4 that the node is using **position control** (coordinate-based control); all other modes (velocity, acceleration, attitude, body_rate) are disabled.

---

### `publish_trajectory_setpoint(self)`
Publishes a `TrajectorySetpoint` message to `/fmu/in/trajectory_setpoint`.
Sends target coordinates in the **NED (North-East-Down)** frame:
- `x = 0.0` (no movement along the North axis)
- `y = 0.0` (no movement along the East axis)
- `z = -takeoff_altitude` (negative because Z points down; ascending requires a negative value)
- `yaw = 0.0` (facing North)
- velocity, acceleration, yawspeed set to `NaN` (PX4 ignores them)

---

### `engage_offboard_mode(self)`
Sends a `DO_SET_MODE` MAVLink command to request PX4 to switch to **OFFBOARD mode**.
Parameters:
- `param1 = 1.0` — enable custom mode flag
- `param2 = 6.0` — OFFBOARD code in PX4 custom mode

---

### `arm(self)`
Sends a `COMPONENT_ARM_DISARM` MAVLink command with `param1 = 1.0` to **arm** (activate motors) the drone.

---

### `disarm(self)`
Sends a `COMPONENT_ARM_DISARM` MAVLink command with `param1 = 0.0` to **disarm** (shut down motors) the drone.
Called automatically when the node is stopped with Ctrl+C.

---

### `publish_vehicle_command(self, command, **params)`
Internal utility function. Packages and publishes a `VehicleCommand` message to `/fmu/in/vehicle_command`.
Arguments:
- `command` — MAVLink command code (e.g. `VEHICLE_CMD_DO_SET_MODE`)
- `**params` — optional parameters `param1` through `param7`

Always sets `target_system = 1`, `target_component = 1`, `from_external = True` so PX4 SITL accepts the command.

---

## `main()` Function

Entry point for the node:
1. Initialize ROS2 (`rclpy.init`)
2. Create a `PX4TakeoffController` instance
3. Run the spin loop to process callbacks and timer
4. On Ctrl+C: call `disarm()`, destroy the node, shut down ROS2

---

## Execution Flow Summary

```
Start
    │
    ├─ [cycles 0–10]   Send offboard mode + setpoint continuously (pre-arm requirement)
    │
    ├─ [cycle 10]      engage_offboard_mode() → PX4 switches to OFFBOARD
    │
    ├─ [cycle 20]      arm() → Motors are activated
    │
    └─ [thereafter]    Continue sending setpoint z = -5.0m → Drone ascends and holds position
```
