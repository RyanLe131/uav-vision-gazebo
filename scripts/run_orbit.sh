#!/bin/bash
# ─────────────────────────────────────────────────────────────
#  run_orbit.sh  —  Build (nếu cần) và chạy orbit_data_collector
#
#  Usage:
#    bash ~/DIPLOM/run_orbit.sh          # chỉ chạy
#    bash ~/DIPLOM/run_orbit.sh --build  # build trước rồi chạy
# ─────────────────────────────────────────────────────────────

set -e

WS=~/DIPLOM/ws_drone
PKG=drone_sim

# ── 1. Build nếu truyền --build ───────────────────────────────
if [[ "$1" == "--build" ]]; then
    echo ""
    echo ">>> Building $PKG ..."
    cd "$WS"
    colcon build --packages-select "$PKG"
    echo ">>> Build xong."
    echo ""
fi

# ── 2. Strip snap khỏi PATH / LD_LIBRARY_PATH (tránh glibc conflict) ──
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v snap | tr '\n' ':' | sed 's/:$//')
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v snap | tr '\n' ':' | sed 's/:$//')
unset PYTHONPATH

# ── 3. Source ROS2 + workspace ────────────────────────────────
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

# ── 4. In thông tin trước khi chạy ───────────────────────────
echo "=========================================="
echo "  orbit_data_collector"
echo "  Workspace : $WS"
echo "  ROS_DISTRO: $ROS_DISTRO"
echo "=========================================="
echo ""

# ── 5. Chạy node ──────────────────────────────────────────────
#exec /usr/bin/python3.12 \
#    "$WS/install/drone_sim/lib/drone_sim/orbit_data_collector"

exec /usr/bin/python3.12 \
    "$WS/install/drone_sim/lib/drone_sim/orbit_detection"
