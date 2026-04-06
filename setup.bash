#!/bin/bash
# =============================================================
# setup.bash — Source ROS2 + workspace environment
#
# Usage:
#   source setup.bash
#
# After sourcing, you can run directly:
#   ros2 launch drone_sim baylands_px4.launch.py
#   ros2 run drone_sim orbit_data_collector
#   colcon build --packages-select drone_sim
# =============================================================

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── 1. Deactivate any venv / conda ──────────────────────────
if [ -n "$VIRTUAL_ENV" ]; then
    echo "[setup] Deactivating venv: $VIRTUAL_ENV"
    deactivate 2>/dev/null || true
fi

if [ -n "$CONDA_DEFAULT_ENV" ] && [ "$CONDA_DEFAULT_ENV" != "base" ]; then
    echo "[setup] Deactivating conda env: $CONDA_DEFAULT_ENV"
    conda deactivate 2>/dev/null || true
fi

# ── 2. Source ROS2 Jazzy ─────────────────────────────────────
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "[setup] ROS2 Jazzy sourced"
else
    echo "[setup] ERROR: /opt/ros/jazzy/setup.bash not found"
    return 1
fi

# ── 3. Source workspace (if already built) ───────────────────
INSTALL_SETUP="$WS_DIR/install/setup.bash"
if [ -f "$INSTALL_SETUP" ]; then
    source "$INSTALL_SETUP"
    echo "[setup] Workspace sourced: $WS_DIR"
else
    echo "[setup] WARN: Workspace not built yet."
    echo "[setup]       Run:  colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3.12"
    echo "[setup]       Then: source setup.bash"
fi

# ── 4. Summary ───────────────────────────────────────────────
echo ""
echo "============================================"
echo "  Environment ready. Quick commands:"
echo ""
echo "  Build (first time or after code changes):"
echo "    colcon build --packages-select drone_sim"
echo ""
echo "  Launch simulation (Terminal 1):"
echo "    ros2 launch drone_sim baylands_px4.launch.py"
echo ""
echo "  Run orbit collector (Terminal 2):"
echo "    ros2 run drone_sim orbit_data_collector"
echo ""
echo "  Check environment:"
echo "    bash check_env.sh"
echo "============================================"
