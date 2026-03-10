#!/usr/bin/env python3
"""
Desktop Reference Card for Drone Bridge Camera System

This file can be printed or displayed for quick reference while working.
Usage: python3 reference_card.py
"""

REFERENCE_CARD = """
╔════════════════════════════════════════════════════════════════════════════╗
║              DRONE BRIDGE CAMERA SYSTEM - QUICK REFERENCE                 ║
╚════════════════════════════════════════════════════════════════════════════╝

📍 WORKSPACE LOCATION
/home/ryanleai/ROSpack/ros2_ws/src/drone_bridge

🎯 MAIN COMMANDS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

VERIFY SYSTEM:
  python3 src/drone_bridge/scripts/verify_setup.py

REBUILD PACKAGE:
  cd ~/ROSpack/ros2_ws
  colcon build --packages-select drone_bridge

LIST TOPICS:
  source install/setup.bash
  ros2 topic list | grep camera

CHECK FRAME RATE:
  ros2 topic hz /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image

VIEW SINGLE FRAME:
  ros2 topic echo /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image --once

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

📚 DOCUMENTATION MAP
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

START HERE (any of these):
  • INDEX.md         - Navigation guide (overview of all docs)
  • QUICKSTART.md    - 5-minute setup (get it running fast)
  • SETUP_COMPLETE.md - Status summary (what's been done)

FOR DETAILS:
  • README.md        - Full reference (500+ lines)
  • ARCHITECTURE.md  - System design (diagrams, topology)
  • EXAMPLES.md      - Code samples (9 practical examples)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

🚀 3-STEP STARTUP (ALWAYS USE THIS ORDER)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

STEP 1 - Terminal #1 (MUST START FIRST):
  cd ~/ROSpack/ardupilot
  ./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-ir --model JSON --console --map
  → Wait for "Ready to fly" message

STEP 2 - Terminal #2 (START AFTER GAZEBO IS RUNNING):
  cd ~/ROSpack/ros2_ws && source install/setup.bash
  ros2 launch drone_bridge gz_bridge.launch.py
  → Wait for "[parameter_bridge] Creating bridge" messages

STEP 3 - Terminal #3 (START AFTER BRIDGE IS RUNNING):
  cd ~/ROSpack/ros2_ws && source install/setup.bash
  python3 install/drone_bridge/share/drone_bridge/scripts/camera_viewer.py
  → Window appears with live camera feed
  → Press 'Q' to quit

✅ SYSTEM WORKING: Live camera feed visible in window!

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

🛠 HELPER TOOLS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

VERIFICATION SCRIPT:
  python3 src/drone_bridge/scripts/verify_setup.py
  → Checks: Python packages, ROS2 packages, Gazebo connection, camera topic

LAUNCH HELPER (from scripts directory):
  cd scripts && source ~/ROSpack/ros2_ws/install/setup.bash
  ./launch_camera.sh bridge    # Launch bridge only
  ./launch_camera.sh viewer    # Launch OpenCV viewer
  ./launch_camera.sh view-rqt  # Launch rqt viewer
  ./launch_camera.sh verify    # Verify system

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

📊 SYSTEM INFORMATION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

GAZEBO WORLD:
  File: ~/ROSpack/ardupilot_gazebo/worlds/iris_runway.sdf
  Model: iris_with_gimbal (with camera + IMU)

CAMERA TOPIC PATH:
  /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
  Message Type: sensor_msgs/msg/Image
  Resolution: 640x480 pixels
  Frame Rate: ~30 Hz
  Encoding: rgb8

ROS2 PACKAGE:
  Location: ~/ROSpack/ros2_ws/src/drone_bridge
  Version: 0.1.0
  Status: Built and tested

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

⚠️  COMMON MISTAKES (AVOID THESE!)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

❌ Starting bridge before Gazebo
   → Bridge will fail to connect
   → ALWAYS start Gazebo first (Terminal 1)

❌ Forgetting to source setup.bash
   → ROS2 commands won't work
   → Run: source ~/ROSpack/ros2_ws/install/setup.bash
   → Add to ~/.bashrc for convenience

❌ Starting camera viewer before bridge
   → Subscriber will wait forever
   → Order: Gazebo → Bridge → Viewer

❌ Running multiple viewers on same topic
   → May cause processing delays
   → Use only one viewer at a time

❌ Not checking Gazebo real_time_factor
   → Low factor = slow/delayed camera feed
   → Check in iris_runway.sdf: real_time_factor should be 1.0

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

🔧 TROUBLESHOOTING FLOW
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

PROBLEM: Camera topic not appearing
  1. Check: ros2 topic list | grep camera
  2. Verify: Gazebo is running (check Terminal 1)
  3. Verify: Bridge is running (check Terminal 2)
  4. Solution: Restart in order (Gazebo → Bridge)
  5. Detailed: See README.md#issue-no-camera-image-topic-appearing

PROBLEM: No display window
  1. Check: DISPLAY environment (echo $DISPLAY)
  2. Try: rqt viewer instead (more reliable)
  3. Solution: export DISPLAY=:0
  4. Detailed: See ARCHITECTURE.md#troubleshooting-decision-tree

PROBLEM: Import errors
  1. Check: source ~/ROSpack/ros2_ws/install/setup.bash
  2. Verify: pip3 list | grep opencv
  3. Solution: pip3 install opencv-python
  4. Detailed: See README.md#troubleshooting

PROBLEM: Low frame rate
  1. Check: ros2 topic hz /world/.../camera/image
  2. Monitor: top or nvidia-smi (system resources)
  3. Solution: Reduce processing in callback
  4. Detailed: See README.md#issue-camera-images-are-delayed-or-slow

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

✨ FILE STRUCTURE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

drone_bridge/
├── [Documentation: 2284 lines total]
│   ├── INDEX.md               [Start here: navigation]
│   ├── QUICKSTART.md          [5-min setup]
│   ├── README.md              [Full reference]
│   ├── ARCHITECTURE.md        [System design]
│   ├── EXAMPLES.md            [9 code examples]
│   └── SETUP_COMPLETE.md      [Status & roadmap]
│
├── [Python: camera streaming]
│   ├── drone_bridge/camera_viewer.py      [OpenCV viewer]
│   ├── drone_bridge/camera_processor.py   [Template]
│   └── drone_bridge/gz_bridge.launch.py   [Bridge launcher]
│
├── [Tools: helpers & diagnostics]
│   ├── scripts/verify_setup.py    [Verification]
│   ├── scripts/launch_camera.sh   [Helper launcher]
│   └── scripts/camera_viewer.py   [Standalone viewer]
│
└── [Configuration]
    ├── package.xml  [ROS2 metadata]
    ├── setup.py     [Python setup]
    └── setup.cfg    [Config]

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

💡 TIPS & TRICKS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

ADD TO ~/.bashrc FOR CONVENIENCE:
  alias bridge_verify='python3 ~/ROSpack/ros2_ws/src/drone_bridge/scripts/verify_setup.py'
  alias source_ros2='source ~/ROSpack/ros2_ws/install/setup.bash'
  alias launch_gz_bridge='cd ~/ROSpack/ros2_ws && source install/setup.bash && ros2 launch drone_bridge gz_bridge.launch.py'

KEEP TERMINALS ORGANIZED:
  Terminal 1: Gazebo (large window)
  Terminal 2: Bridge (left side)
  Terminal 3: Viewer (right side)
  Terminal 4: Commands for testing

MONITOR IN REAL-TIME:
  # In separate terminal
  watch -n 0.5 'ros2 topic hz /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image'

USE SCREEN FOR PERSISTENT TERMINALS:
  screen -S gazebo         # Create named screen
  # Run Gazebo...
  Ctrl+A then D            # Detach
  screen -r gazebo         # Reattach later

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

📞 WHEN YOU GET STUCK
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

1. Run verification script:
   python3 src/drone_bridge/scripts/verify_setup.py

2. Check system status:
   ros2 topic list
   ros2 topic hz /world/.../camera/image

3. Review relevant documentation:
   • For setup: QUICKSTART.md
   • For errors: README.md#troubleshooting
   • For architecture: ARCHITECTURE.md
   • For code: EXAMPLES.md

4. Enable debug logging:
   ros2 launch drone_bridge gz_bridge.launch.py --log-level DEBUG

5. Check logs:
   cat ~/.ros/log/latest/*/stdout.log

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

🎯 SUCCESS CRITERIA
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

✅ System is working when:
   • Verification script shows all checks PASS
   • ros2 topic list shows camera topic
   • ros2 topic hz shows ~30 Hz frame rate
   • Camera viewer window displays live feed
   • Can press 'Q' to quit without errors

✅ Ready for image processing when:
   • Above criteria met
   • Can run example from EXAMPLES.md
   • Custom node can subscribe to camera topic
   • Processing doesn't cause frame drops

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Generated: March 6, 2026
Package: drone_bridge v0.1.0
Status: ✅ Complete and tested
═══════════════════════════════════════════════════════════════════════════════
"""

if __name__ == '__main__':
    print(REFERENCE_CARD)
    
    # Save to file
    with open('REFERENCE_CARD.txt', 'w') as f:
        f.write(REFERENCE_CARD)
    
    print("\n✅ Reference card saved to: REFERENCE_CARD.txt")
