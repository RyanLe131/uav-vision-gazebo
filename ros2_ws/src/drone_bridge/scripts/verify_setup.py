#!/usr/bin/env python3
"""
Setup and Verification Script for Drone Bridge Camera System

This script helps you:
1. Verify all dependencies are installed
2. Check if ROS2 packages are properly configured
3. Test the camera topic is working
4. Provide diagnostic information

Usage:
    python3 verify_setup.py
    python3 verify_setup.py --check-topics
    python3 verify_setup.py --diagnose
"""

import subprocess
import sys
from pathlib import Path


class Colors:
    """ANSI color codes for terminal output."""
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    END = '\033[0m'


def run_command(cmd, shell=False):
    """
    Run a shell command and return output.
    
    Args:
        cmd: Command to run (str or list)
        shell: Whether to use shell (bool)
    
    Returns:
        tuple: (success, output, error)
    """
    try:
        result = subprocess.run(
            cmd,
            shell=shell,
            capture_output=True,
            text=True,
            timeout=5
        )
        return result.returncode == 0, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return False, "", "Command timed out"
    except Exception as e:
        return False, "", str(e)


def check_python_packages():
    """Check if required Python packages are installed."""
    print(f"\n{Colors.BLUE}Checking Python packages...{Colors.END}")
    
    packages = {
        'rclpy': 'ROS2 Python client',
        'cv_bridge': 'OpenCV-ROS bridge',
        'cv2': 'OpenCV',
        'sensor_msgs': 'ROS2 sensor messages',
    }
    
    all_ok = True
    for pkg, desc in packages.items():
        try:
            __import__(pkg)
            print(f"  {Colors.GREEN}✓{Colors.END} {pkg:<20} - {desc}")
        except ImportError:
            print(f"  {Colors.RED}✗{Colors.END} {pkg:<20} - {desc} (NOT INSTALLED)")
            all_ok = False
    
    return all_ok


def check_ros2_packages():
    """Check if required ROS2 packages are installed."""
    print(f"\n{Colors.BLUE}Checking ROS2 packages...{Colors.END}")
    
    packages = [
        'ros_gz_bridge',
        'rqt_image_view',
        'cv_bridge',
        'sensor_msgs',
    ]
    
    all_ok = True
    for pkg in packages:
        success, output, error = run_command(
            f"ros2 pkg list | grep -w {pkg}",
            shell=True
        )
        if success and output.strip():
            print(f"  {Colors.GREEN}✓{Colors.END} {pkg:<25} - Found")
        else:
            print(f"  {Colors.RED}✗{Colors.END} {pkg:<25} - Not found")
            all_ok = False
    
    return all_ok


def check_camera_topic():
    """Check if camera topic is available."""
    print(f"\n{Colors.BLUE}Checking camera topic...{Colors.END}")
    
    camera_topic = (
        '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/'
        'pitch_link/sensor/camera/image'
    )
    
    success, output, error = run_command(
        f"ros2 topic list | grep camera",
        shell=True
    )
    
    if success and 'camera' in output:
        print(f"  {Colors.GREEN}✓{Colors.END} Camera topic found")
        print(f"      Topic: {camera_topic}")
        
        # Check topic info
        success, info_output, error = run_command(
            ['ros2', 'topic', 'info', camera_topic],
            shell=False
        )
        if success:
            print(f"      {Colors.GREEN}✓{Colors.END} Topic info available")
            for line in info_output.split('\n'):
                if line.strip():
                    print(f"          {line}")
        return True
    else:
        print(f"  {Colors.RED}✗{Colors.END} Camera topic NOT found")
        print(f"      Expected: {camera_topic}")
        print(f"      Make sure Gazebo is running and gazebo-ros-bridge is launched")
        return False


def check_gazebo():
    """Check if Gazebo is running."""
    print(f"\n{Colors.BLUE}Checking Gazebo connection...{Colors.END}")
    
    success, output, error = run_command(
        "ros2 topic list | grep -E '^/world/'",
        shell=True
    )
    
    if success and output.strip():
        topic_count = len(output.strip().split('\n'))
        print(f"  {Colors.GREEN}✓{Colors.END} Gazebo is running")
        print(f"      Found {topic_count} world topics")
        return True
    else:
        print(f"  {Colors.YELLOW}⚠{Colors.END} Gazebo does not appear to be running")
        print(f"      Run: ./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-ir")
        return False


def verify_setup():
    """Run all verification checks."""
    print(f"\n{Colors.BLUE}{'='*60}")
    print(f"Drone Bridge Camera System - Setup Verification")
    print(f"{'='*60}{Colors.END}")
    
    results = {}
    
    # Run all checks
    results['python'] = check_python_packages()
    results['ros2'] = check_ros2_packages()
    results['gazebo'] = check_gazebo()
    results['camera'] = check_camera_topic()
    
    # Summary
    print(f"\n{Colors.BLUE}{'='*60}")
    print(f"Summary{Colors.END}")
    print(f"{'='*60}")
    
    all_ok = all(results.values())
    
    for check, result in results.items():
        status = f"{Colors.GREEN}✓ PASS{Colors.END}" if result else f"{Colors.RED}✗ FAIL{Colors.END}"
        print(f"  {check:<20} {status}")
    
    print(f"{'='*60}\n")
    
    if all_ok:
        print(f"{Colors.GREEN}All checks passed! Your system is ready to use.{Colors.END}")
        print(f"\nNext steps:")
        print(f"  1. Launch the bridge: ros2 launch drone_bridge gz_bridge.launch.py")
        print(f"  2. Run camera viewer: python3 ...install/drone_bridge/share/drone_bridge/scripts/camera_viewer.py")
    else:
        print(f"{Colors.RED}Some checks failed. See above for details.{Colors.END}")
        print(f"\nCommon solutions:")
        print(f"  - Install missing packages: sudo apt-get install ros-humble-ros-gz-bridge")
        print(f"  - Install OpenCV: pip3 install opencv-python")
        print(f"  - Start Gazebo: ./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-ir")
    
    return all_ok


if __name__ == '__main__':
    if len(sys.argv) > 1:
        if sys.argv[1] == '--check-topics':
            check_camera_topic()
        elif sys.argv[1] == '--diagnose':
            verify_setup()
    else:
        success = verify_setup()
        sys.exit(0 if success else 1)
