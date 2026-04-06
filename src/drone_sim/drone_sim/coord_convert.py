"""
coord_convert.py
================
Coordinate system conversions used in the drone project:

  - ENU  (East-North-Up)   : Gazebo / SDF / ROS frame
  - NED  (North-East-Down) : PX4 / MAVLink frame

Conversion rules:
  ENU -> NED:   x_ned =  y_enu
                y_ned =  x_enu
                z_ned = -z_enu

  NED -> ENU:   x_enu =  y_ned
                y_enu =  x_ned
                z_enu = -z_ned

Example from this project:
  person_1 in baylands.sdf : <pose>10 0 1.0 ...</pose>  (ENU)
    -> ENU : x=10,  y=0,   z=1.0
    -> NED : x=0,   y=10,  z=-1.0

Usage:
  python3 coord_convert.py
"""


# ── Simple data type ──────────────────────────────────────────────────────────

class Point3D:
    """Three-dimensional point x, y, z."""
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return f"({self.x}, {self.y}, {self.z})"


# ── Conversion functions ──────────────────────────────────────────────────────

def enu_to_ned(x_enu: float, y_enu: float, z_enu: float) -> Point3D:
    """
    Convert coordinates from ENU to NED.

    ENU (Gazebo/SDF):
        x -> East
        y -> North
        z -> Up

    NED (PX4):
        x -> North
        y -> East
        z -> Down  (negative = above ground)

    Formula:
        x_ned =  y_enu
        y_ned =  x_enu
        z_ned = -z_enu
    """
    return Point3D(
        x =  y_enu,
        y =  x_enu,
        z = -z_enu,
    )


def ned_to_enu(x_ned: float, y_ned: float, z_ned: float) -> Point3D:
    """
    Convert coordinates from NED to ENU.

    Formula:
        x_enu =  y_ned
        y_enu =  x_ned
        z_enu = -z_ned
    """
    return Point3D(
        x =  y_ned,
        y =  x_ned,
        z = -z_ned,
    )


# ── Demo / self-test ─────────────────────────────────────────────────────────

if __name__ == '__main__':

    print("=" * 55)
    print("  COORDINATE CONVERSION TABLE ENU <-> NED")
    print("=" * 55)

    persons = [
        ("person_1 (Male visitor)",      10,  0, 1.0),
        ("person_2 (Nurse Female)",       15,  0, 0.0),
        ("person_3 (Patient Walking)",    20,  0, 0.0),
    ]

    print(f"\n{'Name':<30} {'ENU (Gazebo/SDF)':<22} {'NED (PX4)':<22}")
    print("-" * 75)

    for name, ex, ey, ez in persons:
        ned = enu_to_ned(ex, ey, ez)
        print(f"{name:<30} x={ex:>5} y={ey:>5} z={ez:>4}    "
              f"x={ned.x:>5} y={ned.y:>5} z={ned.z:>5}")

    print()
    print("=" * 55)
    print("  ROUND-TRIP CHECK (ENU -> NED -> ENU)")
    print("=" * 55)
    test_x, test_y, test_z = 10.0, 0.0, 1.0
    ned  = enu_to_ned(test_x, test_y, test_z)
    back = ned_to_enu(ned.x, ned.y, ned.z)
    print(f"\n  Original ENU : x={test_x}, y={test_y}, z={test_z}")
    print(f"  -> NED       : {ned}")
    print(f"  -> ENU back  : {back}")
    ok = (back.x == test_x and back.y == test_y and back.z == test_z)
    print(f"  Result       : {'OK' if ok else 'FAIL'}")

    print()
    print("=" * 55)
    print("  APPLICATION: ORBIT AROUND PERSON_1")
    print("=" * 55)
    print()
    print("  person_1 in SDF (ENU): x=10, y=0, z=1.0")
    ned_p1 = enu_to_ned(10, 0, 1.0)
    print(f"  -> NED (used in Python): x={ned_p1.x}, y={ned_p1.y}")
    print()
    print("  Orbit start point (radius=5m, angle=0):")
    import math
    RADIUS = 5.0
    start_ned_x = ned_p1.x + RADIUS * math.cos(0)
    start_ned_y = ned_p1.y + RADIUS * math.sin(0)
    print(f"    NED: x={start_ned_x}, y={start_ned_y}")
    start_enu = ned_to_enu(start_ned_x, start_ned_y, 0)
    print(f"    ENU: x={start_enu.x}, y={start_enu.y}")
    print()
