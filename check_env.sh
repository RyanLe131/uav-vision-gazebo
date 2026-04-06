#!/bin/bash
# ============================================================
# check_env.sh — Check environment before running simulation
# Usage: bash check_env.sh
# ============================================================

WS=/home/ryan-le-ai/DIPLOM/ws_drone
PASS=0
FAIL=0

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

ok()   { echo -e "  ${GREEN}[OK]${NC}   $1"; ((PASS++)); }
fail() { echo -e "  ${RED}[FAIL]${NC} $1"; ((FAIL++)); }
warn() { echo -e "  ${YELLOW}[WARN]${NC} $1"; }

echo ""
echo "======================================================"
echo "  Environment Check — DIPLOM Drone Simulation"
echo "======================================================"

# ── 1. Python version ─────────────────────────────────────
echo ""
echo "[ 1 ] Python"
PY=$(python3 --version 2>&1)
if echo "$PY" | grep -q "3.12"; then
    ok "Python: $PY"
else
    fail "Python: $PY  (requires 3.12, do not use conda/venv)"
fi

# Warn if conda is active
if [ -n "$CONDA_DEFAULT_ENV" ] && [ "$CONDA_DEFAULT_ENV" != "base" ]; then
    fail "Conda env is active: $CONDA_DEFAULT_ENV — run 'conda deactivate' first"
fi
if [ -n "$VIRTUAL_ENV" ]; then
    fail "venv is active: $VIRTUAL_ENV — run 'deactivate' first"
fi

# ── 2. ROS2 sourced ───────────────────────────────────────
echo ""
echo "[ 2 ] ROS2 Jazzy"
if [ -n "$ROS_DISTRO" ] && [ "$ROS_DISTRO" = "jazzy" ]; then
    ok "ROS_DISTRO=$ROS_DISTRO"
else
    fail "ROS2 Jazzy not sourced — run: source /opt/ros/jazzy/setup.bash"
fi

# ── 3. Workspace sourced ──────────────────────────────────
echo ""
echo "[ 3 ] Workspace"
if echo "$AMENT_PREFIX_PATH" | grep -q "$WS/install"; then
    ok "Workspace sourced: $WS"
else
    fail "Workspace not sourced — run: source $WS/install/setup.bash"
fi

# ── 4. Python packages import ─────────────────────────────
echo ""
echo "[ 4 ] Python imports"

python3 -c "import rclpy" 2>/dev/null \
    && ok "rclpy" || fail "rclpy could not be imported"

python3 -c "from px4_msgs.msg import OffboardControlMode" 2>/dev/null \
    && ok "px4_msgs" || fail "px4_msgs could not be imported — rebuild with: colcon build --packages-select px4_msgs --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3.12"

python3 -c "from px4_msgs.msg import VehicleLocalPosition" 2>/dev/null \
    && ok "px4_msgs.VehicleLocalPosition" || fail "px4_msgs.VehicleLocalPosition"

python3 -c "import cv2" 2>/dev/null \
    && ok "opencv (cv2)" || fail "opencv not installed — sudo apt install python3-opencv"

python3 -c "from cv_bridge import CvBridge" 2>/dev/null \
    && ok "cv_bridge" || fail "cv_bridge not installed — sudo apt install ros-jazzy-cv-bridge"

# ── 5. ROS2 packages ──────────────────────────────────────
echo ""
echo "[ 5 ] ROS2 packages"

ros2 pkg list 2>/dev/null | grep -q "drone_sim" \
    && ok "drone_sim" || fail "drone_sim not built — colcon build --packages-select drone_sim"

ros2 pkg list 2>/dev/null | grep -q "px4_msgs" \
    && ok "px4_msgs (ros2 pkg)" || fail "px4_msgs not found in ros2 pkg list"

ros2 pkg list 2>/dev/null | grep -q "px4_ros_com" \
    && ok "px4_ros_com (ros2 pkg)" || fail "px4_ros_com not found in ros2 pkg list"

# ── 6. Python version in install/ ─────────────────────────
echo ""
echo "[ 6 ] Build artifacts"

if [ -d "$WS/install/px4_msgs/lib/python3.12" ]; then
    ok "px4_msgs built with python3.12"
elif [ -d "$WS/install/px4_msgs/lib/python3.13" ]; then
    fail "px4_msgs built with python3.13 — rebuild required!"
else
    fail "px4_msgs install not found"
fi

if [ -d "$WS/install/px4_ros_com/lib/python3.12" ]; then
    ok "px4_ros_com built with python3.12"
elif [ -d "$WS/install/px4_ros_com/lib/python3.13" ]; then
    fail "px4_ros_com built with python3.13 — rebuild required!"
else
    warn "px4_ros_com has no python lib (normal if no Python nodes)"
fi

# ── 7. PX4 Autopilot ─────────────────────────────────────
echo ""
echo "[ 7 ] PX4 Autopilot"
PX4_DIR="$HOME/PX4-Autopilot"
if [ -d "$PX4_DIR" ]; then
    ok "PX4-Autopilot found: $PX4_DIR"
else
    fail "PX4-Autopilot not found at $PX4_DIR"
fi

PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
if [ -f "$PX4_BIN" ]; then
    ok "PX4 binary built: $PX4_BIN"
else
    fail "PX4 binary not built — run: cd $PX4_DIR && make px4_sitl gz_x500_mono_cam"
fi

# ── 8. MicroXRCEAgent ─────────────────────────────────────
echo ""
echo "[ 8 ] MicroXRCEAgent"
which MicroXRCEAgent > /dev/null 2>&1 \
    && ok "MicroXRCEAgent: $(which MicroXRCEAgent)" \
    || fail "MicroXRCEAgent not found — installation required"

# ── Summary ───────────────────────────────────────────────
echo ""
echo "======================================================"
if [ $FAIL -eq 0 ]; then
    echo -e "  ${GREEN}ALL OK ($PASS/$((PASS+FAIL))) — Ready to run simulation!${NC}"
    echo ""
    echo "  Launch order:"
    echo "    Terminal 1: cd ~/PX4-Autopilot && make px4_sitl gz_x500_mono_cam"
    echo "    Terminal 2: MicroXRCEAgent udp4 -p 8888"
    echo "    Terminal 3: ros2 run drone_sim orbit_data_collector"
else
    echo -e "  ${RED}ERRORS FOUND: $FAIL failed, $PASS OK — Fix before running!${NC}"
fi
echo "======================================================"
echo ""
