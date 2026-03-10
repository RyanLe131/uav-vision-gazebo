import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ryanleai/ROSpack/ros2_ws/install/drone_bridge'
