#!/bin/bash

# Drone Bridge Launcher Script
# This script simplifies launching the camera viewer system

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
ROS_DISTRO="humble"

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Help function
show_help() {
    cat << EOF
Drone Bridge Camera System Launcher

Usage: ./launch_camera.sh [OPTION]

Options:
    bridge              Launch only the Gazebo-ROS2 bridge
    viewer              Launch camera viewer (requires bridge running)
    view-rqt            Launch rqt image viewer (requires bridge running)
    verify              Verify system setup
    help                Show this help message

Examples:
    # Terminal 1: Launch bridge
    ./launch_camera.sh bridge

    # Terminal 2: Launch OpenCV viewer
    ./launch_camera.sh viewer

    # Or use rqt viewer
    ./launch_camera.sh view-rqt

    # Check if everything is set up correctly
    ./launch_camera.sh verify

EOF
}

# Source ROS2
source_ros2() {
    if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
        source "/opt/ros/$ROS_DISTRO/setup.bash"
    else
        echo -e "${RED}Error: Could not find ROS2 $ROS_DISTRO installation${NC}"
        return 1
    fi
    
    if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
        source "$WORKSPACE_DIR/install/setup.bash"
    else
        echo -e "${RED}Error: Could not find workspace setup.bash${NC}"
        echo "Make sure the package is built: cd $WORKSPACE_DIR && colcon build"
        return 1
    fi
}

# Launch bridge
launch_bridge() {
    echo -e "${BLUE}Launching Gazebo-ROS2 Bridge...${NC}"
    echo "Topics that will be available:"
    echo "  - /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"
    echo "  - /world/iris_runway/model/iris_with_gimbal/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu"
    echo ""
    echo -e "${YELLOW}Note: Make sure Gazebo is running in another terminal${NC}"
    echo ""
    
    source_ros2 || return 1
    ros2 launch drone_bridge gz_bridge.launch.py
}

# Launch viewer
launch_viewer() {
    echo -e "${BLUE}Launching OpenCV Camera Viewer...${NC}"
    echo "Window: Gazebo Camera Feed"
    echo "Press 'Q' to quit"
    echo ""
    
    source_ros2 || return 1
    python3 "$WORKSPACE_DIR/install/drone_bridge/share/drone_bridge/scripts/camera_viewer.py"
}

# Launch rqt viewer
launch_view_rqt() {
    echo -e "${BLUE}Launching rqt Image Viewer...${NC}"
    echo ""
    
    source_ros2 || return 1
    ros2 launch drone_bridge camera_view.launch.py
}

# Verify setup
verify_setup() {
    echo -e "${BLUE}Verifying Drone Bridge Setup...${NC}"
    echo ""
    
    source_ros2 || return 1
    python3 "$WORKSPACE_DIR/src/drone_bridge/scripts/verify_setup.py"
}

# Main
if [ $# -eq 0 ]; then
    show_help
    exit 0
fi

case "$1" in
    bridge)
        launch_bridge
        ;;
    viewer)
        launch_viewer
        ;;
    view-rqt)
        launch_view_rqt
        ;;
    verify)
        verify_setup
        ;;
    help|-h|--help)
        show_help
        ;;
    *)
        echo -e "${RED}Unknown option: $1${NC}"
        show_help
        exit 1
        ;;
esac
