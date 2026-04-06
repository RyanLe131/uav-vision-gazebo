import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    """
    Full autonomous launch — single command, no manual intervention.

    Sequence (all automatic):
      t=0s   Gazebo server + GUI
      t=0s   MicroXRCE-DDS Agent (udp4 port 8888)
      t=8s   PX4 SITL (wait for Gazebo to load world)
      t=20s  Camera bridge + Clock bridge (wait for PX4 to spawn drone)
      t=30s  orbit_data_collector node (wait for bridge + EKF2 init)
             -> node waits for EKF2 ready -> OFFBOARD -> ARM -> TAKEOFF -> ORBIT -> LAND
             -> auto-exits after 120s or after 3 laps

    Usage:
      source /opt/ros/jazzy/setup.bash
      source ~/DIPLOM/ws_drone/install/setup.bash
      ros2 launch drone_sim baylands_px4.launch.py
    """

    px4_dir       = os.path.expanduser('~/PX4-Autopilot')
    px4_build_dir = os.path.join(px4_dir, 'build/px4_sitl_default')
    px4_bin       = os.path.join(px4_build_dir, 'bin/px4')
    px4_rootfs    = os.path.join(px4_build_dir, 'rootfs')
    px4_models    = os.path.join(px4_dir, 'Tools/simulation/gz/models')
    px4_worlds    = os.path.join(px4_dir, 'Tools/simulation/gz/worlds')
    px4_plugins   = os.path.join(px4_build_dir, 'src/modules/simulation/gz_plugins')

    world_file = os.path.join(px4_worlds, 'baylands.sdf')

    gz_env = {
        'GZ_SIM_RESOURCE_PATH':      f'{px4_models}:{px4_worlds}',
        'GZ_SIM_SYSTEM_PLUGIN_PATH': px4_plugins,
    }

    # Strip snap paths from PATH/LD_LIBRARY_PATH to avoid glibc conflicts in GUI
    clean_path = ':'.join(p for p in os.environ.get('PATH', '').split(':')
                          if 'snap' not in p and p)
    clean_ld   = ':'.join(p for p in os.environ.get('LD_LIBRARY_PATH', '').split(':')
                          if 'snap' not in p and p)

    # ── 1. Gazebo server ──────────────────────────────────────────────────────
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '--verbose=1', '-r', '-s', world_file],
        additional_env=gz_env,
        output='screen',
    )

    # ── 2. Gazebo GUI ─────────────────────────────────────────────────────────
    gz_gui = ExecuteProcess(
        cmd=['bash', '-c',
             f'export PATH="{clean_path}" && '
             f'export LD_LIBRARY_PATH="{clean_ld}" && '
             'unset GTK_PATH GTK_EXE_PREFIX GIO_MODULE_DIR LOCPATH && '
             'exec gz sim -v 1 -g'],
        output='screen',
    )

    # ── 3. MicroXRCE-DDS Agent ────────────────────────────────────────────────
    # Kill old MicroXRCEAgent + gz processes first to avoid "bind error port 8888"
    # then start a fresh agent
    microxrce = ExecuteProcess(
        cmd=['bash', '-c',
             'pkill -f MicroXRCEAgent 2>/dev/null; '
             'pkill -f "gz sim" 2>/dev/null; '
             'sleep 1 && '
             'exec MicroXRCEAgent udp4 -p 8888'],
        output='screen',
    )

    # ── 4. PX4 SITL ───────────────────────────────────────────────────────────
    px4_sitl = ExecuteProcess(
        cmd=['bash', '-c',
             f'cd {px4_rootfs} && '
             f'PX4_SYS_AUTOSTART=4010 '
             f'PX4_SIM_MODEL=gz_x500_mono_cam '
             f'PX4_GZ_WORLD=baylands '
             f'PX4_GZ_STANDALONE=1 '
             f'PX4_GZ_MODEL_POSE="0,0,0,0,0,0" '
             f'{px4_bin} -w {px4_rootfs}'],
        additional_env=gz_env,
        output='screen',
    )

    # ── 5. Camera bridge ──────────────────────────────────────────────────────
    gz_cam_topic = (
        '/world/baylands/model/x500_mono_cam_0'
        '/link/camera_link/sensor/camera/image'
    )
    bridge_camera = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[gz_cam_topic],
        remappings=[(gz_cam_topic, '/camera/image_raw')],
        output='screen',
    )

    # ── 6. Clock bridge ───────────────────────────────────────────────────────
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    return LaunchDescription([
        gz_server,
        gz_gui,
        microxrce,
        # PX4 waits 8s for Gazebo to load world + actors
        TimerAction(period=8.0,  actions=[px4_sitl]),
        # Bridge waits 20s for PX4 to spawn drone
        TimerAction(period=20.0, actions=[bridge_camera, bridge_clock]),
        # orbit_data_collector is run separately via run_orbit.sh after launch stabilizes
    ])
