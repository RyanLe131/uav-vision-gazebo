#!/usr/bin/env python3
"""
drone_controller.py

Drone control module — fully decoupled from ROS2.
Can be imported and used from any node.

Contains 2 controllers:
  1. PurePursuitOrbit  — computes next setpoint on a circular orbit
  2. PIDController     — compensates for error (e.g. pixel error from camera)

Coordinates: NED (North-East-Down) — PX4 standard
  x = North, y = East, z = Down (negative = above ground)

Usage:
    from drone_sim.drone_controller import PurePursuitOrbit, PIDController

    orbit = PurePursuitOrbit(
        center_x=0.0, center_y=10.0,
        radius=9.0, altitude=5.0,
        speed=0.3,           # rad/s
        lookahead=1.5,       # m
    )

    pid_x = PIDController(kp=0.4, ki=0.01, kd=0.1, output_limit=2.0)
    pid_y = PIDController(kp=0.4, ki=0.01, kd=0.1, output_limit=2.0)

    # In the control loop (20 Hz):
    sp = orbit.step(drone_x, drone_y, dt=0.05)
    correction_x = pid_x.update(pixel_error_x, dt=0.05)
    correction_y = pid_y.update(pixel_error_y, dt=0.05)
"""

import math
from dataclasses import dataclass


# ── Data classes ──────────────────────────────────────────────────────────────

@dataclass
class Setpoint:
    """Position setpoint sent to PX4 (NED)."""
    x:   float   # North (m)
    y:   float   # East  (m)
    z:   float   # Down  (m, negative = above ground)
    yaw: float   # rad, NED yaw (0 = North, increasing clockwise)


@dataclass
class PIDState:
    """Internal state of a single PID controller."""
    integral:   float = 0.0
    prev_error: float = 0.0


# ── PID Controller ────────────────────────────────────────────────────────────

class PIDController:
    """
    Single-axis PID controller.

    Used to compensate for small errors — e.g. pixel error from camera,
    or distance deviation from target.

    Args:
        kp: Proportional gain
        ki: Integral gain
        kd: Derivative gain
        output_limit: Output clamp limit (±output_limit)
        integral_limit: Integral clamp to prevent windup
    """

    def __init__(self,
                 kp: float = 0.5,
                 ki: float = 0.0,
                 kd: float = 0.1,
                 output_limit: float = 2.0,
                 integral_limit: float = 5.0):
        self.kp             = kp
        self.ki             = ki
        self.kd             = kd
        self.output_limit   = output_limit
        self.integral_limit = integral_limit
        self._state         = PIDState()

    def update(self, error: float, dt: float) -> float:
        """
        Compute output from error and dt.

        Args:
            error: Current error (target - current)
            dt:    Time since last update (s)

        Returns:
            Correction value (clamped to ±output_limit)
        """
        if dt <= 0:
            return 0.0

        # Proportional
        p = self.kp * error

        # Integral (with anti-windup)
        self._state.integral += error * dt
        self._state.integral  = _clamp(
            self._state.integral, -self.integral_limit, self.integral_limit)
        i = self.ki * self._state.integral

        # Derivative
        d = self.kd * (error - self._state.prev_error) / dt
        self._state.prev_error = error

        output = p + i + d
        return _clamp(output, -self.output_limit, self.output_limit)

    def reset(self):
        """Reset PID state (call when switching flight state)."""
        self._state = PIDState()

    def set_gains(self, kp: float, ki: float, kd: float):
        """Update gains at runtime."""
        self.kp = kp
        self.ki = ki
        self.kd = kd


# ── Pure Pursuit Orbit ────────────────────────────────────────────────────────

class PurePursuitOrbit:
    """
    Pure Pursuit controller for circular orbit.

    Principle:
      - Drone is at angle θ on the circle
      - Compute a "lookahead" point ahead of the drone on the circle
      - Setpoint = that lookahead point
      - Yaw = direction toward center (center_x, center_y)

    Advantages over fixed-angle stepping:
      - Automatically smooth when drone drifts off the orbit
      - No sudden jumps when drone hasn't reached the previous setpoint

    Args:
        center_x:  NED North coordinate of orbit center (m)
        center_y:  NED East coordinate of orbit center (m)
        radius:    Orbit radius (m)
        altitude:  Flight altitude (m, positive = above ground)
        speed:     Angular speed (rad/s, positive = counter-clockwise in NED)
        lookahead: Lookahead distance on the circle (m)
                   Converted to angle internally: lookahead_angle = lookahead / radius
    """

    def __init__(self,
                 center_x:  float = 0.0,
                 center_y:  float = 10.0,
                 radius:    float = 9.0,
                 altitude:  float = 5.0,
                 speed:     float = 0.3,
                 lookahead: float = 1.5):
        self.center_x  = center_x
        self.center_y  = center_y
        self.radius    = radius
        self.altitude  = altitude
        self.speed     = speed          # rad/s
        self.lookahead = lookahead      # m

        # Current angle on the circle (rad)
        self._angle        = 0.0
        # Accumulated angle for lap counting
        self._angle_acc    = 0.0
        self._laps_done    = 0

    @property
    def laps_done(self) -> int:
        return self._laps_done

    @property
    def current_angle_deg(self) -> float:
        return math.degrees(self._angle) % 360

    def reset(self, start_angle: float = 0.0):
        """Reset to initial angle."""
        self._angle     = start_angle
        self._angle_acc = 0.0
        self._laps_done = 0

    def init_angle_from_position(self, drone_x: float, drone_y: float):
        """
        Initialize angle from the drone's current position.
        Call once before starting orbit to avoid a sudden jump.
        """
        dx = drone_x - self.center_x
        dy = drone_y - self.center_y
        self._angle = math.atan2(dy, dx)

    def step(self, drone_x: float, drone_y: float, dt: float) -> Setpoint:
        """
        Compute the next setpoint.

        Args:
            drone_x: Current drone NED North position (m)
            drone_y: Current drone NED East position (m)
            dt:      Time since last call (s)

        Returns:
            NED Setpoint to send to PX4
        """
        # ── Advance angle ─────────────────────────────────────────────────────
        delta = self.speed * dt
        self._angle     += delta
        self._angle_acc += delta

        # Lap counting
        while self._angle_acc >= 2.0 * math.pi:
            self._angle_acc -= 2.0 * math.pi
            self._laps_done += 1

        # ── Pure Pursuit: lookahead point ─────────────────────────────────────
        lookahead_angle = self.lookahead / self.radius   # rad
        target_angle    = self._angle + lookahead_angle

        sp_x = self.center_x + self.radius * math.cos(target_angle)
        sp_y = self.center_y + self.radius * math.sin(target_angle)
        sp_z = -self.altitude   # NED: negative = above ground

        # ── Yaw: always face orbit center ────────────────────────────────────
        yaw = self._yaw_to_center(sp_x, sp_y)

        return Setpoint(x=sp_x, y=sp_y, z=sp_z, yaw=yaw)

    def distance_to_orbit(self, drone_x: float, drone_y: float) -> float:
        """Distance from drone to orbit circle (m). 0 = on the circle."""
        dist_to_center = math.sqrt(
            (drone_x - self.center_x)**2 + (drone_y - self.center_y)**2)
        return abs(dist_to_center - self.radius)

    def _yaw_to_center(self, from_x: float, from_y: float) -> float:
        """NED yaw pointing toward center from point (from_x, from_y)."""
        dx = self.center_x - from_x
        dy = self.center_y - from_y
        return math.atan2(dy, dx)


# ── Camera PID (pixel error -> position correction) ───────────────────────────

class CameraTrackingPID:
    """
    Use pixel error from detection to adjust drone position.

    When YOLO detects a person:
      pixel_error_x = (bbox_center_x - frame_width/2)  / frame_width
      pixel_error_y = (bbox_center_y - frame_height/2) / frame_height
    Both in [-0.5, 0.5] — normalized

    Controller computes correction (m) to add to the setpoint from PurePursuitOrbit.

    Args:
        kp, ki, kd: Gains for both axes
        output_limit: Correction clamp (m)
    """

    def __init__(self,
                 kp: float = 1.5,
                 ki: float = 0.0,
                 kd: float = 0.3,
                 output_limit: float = 1.5):
        self._pid_x = PIDController(kp, ki, kd, output_limit)
        self._pid_y = PIDController(kp, ki, kd, output_limit)
        self._active = False

    def update(self,
               pixel_err_x: float,
               pixel_err_y: float,
               dt: float,
               yaw: float) -> tuple[float, float]:
        """
        Compute NED correction from pixel error.

        pixel_err_x > 0 -> person shifted right  -> drone moves right
        pixel_err_y > 0 -> person shifted down   -> altitude not adjusted here

        Args:
            pixel_err_x: (bbox_cx - frame_w/2) / frame_w  in [-0.5, 0.5]
            pixel_err_y: (bbox_cy - frame_h/2) / frame_h  in [-0.5, 0.5]
            dt:          time step (s)
            yaw:         current drone yaw (rad NED) — used to rotate correction

        Returns:
            (corr_x, corr_y) — NED correction (m) to add to setpoint
        """
        self._active = True

        # PID on pixel error (camera frame)
        corr_cam_x = self._pid_x.update(pixel_err_x, dt)
        corr_cam_y = self._pid_y.update(pixel_err_y, dt)

        # Rotate from camera frame -> NED based on drone yaw
        # Camera faces yaw direction -> pixel_x offset = East/West offset
        corr_ned_x = -math.sin(yaw) * corr_cam_x
        corr_ned_y =  math.cos(yaw) * corr_cam_x

        return corr_ned_x, corr_ned_y

    def reset(self):
        self._pid_x.reset()
        self._pid_y.reset()
        self._active = False


# ── Utility ───────────────────────────────────────────────────────────────────

def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_pi(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle >  math.pi: angle -= 2 * math.pi
    while angle < -math.pi: angle += 2 * math.pi
    return angle


def compute_pixel_error(bbox_cx: float, bbox_cy: float,
                        frame_w: float, frame_h: float) -> tuple[float, float]:
    """
    Compute normalized pixel error from bounding box center.

    Returns:
        (err_x, err_y) in [-0.5, 0.5]
        err_x > 0 -> person shifted right
        err_y > 0 -> person shifted down
    """
    err_x = (bbox_cx - frame_w / 2.0) / frame_w
    err_y = (bbox_cy - frame_h / 2.0) / frame_h
    return err_x, err_y
