import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'drone_sim'

def get_model_data_files():
    """Collect all files from models/ subdirectories for installation."""
    data_files = []
    models_dir = 'models'
    if os.path.isdir(models_dir):
        for dirpath, dirnames, filenames in os.walk(models_dir, followlinks=False):
            if filenames:
                install_dir = os.path.join('share', package_name, dirpath)
                # Skip symlinks — they cause errors with setuptools copy
                files = [
                    os.path.join(dirpath, f) for f in filenames
                    if not os.path.islink(os.path.join(dirpath, f))
                ]
                if files:
                    data_files.append((install_dir, files))
    return data_files

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*'))),
    ] + get_model_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryan-le-ai',
    maintainer_email='ryan-le-ai@todo.todo',
    description='ROS2 Drone Simulation with Gazebo Iris Model',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'px4_takeoff_controller = drone_sim.px4_takeoff_controller:main',
            'car_mover = drone_sim.car_mover:main',
            'image_collector = drone_sim.image_collector:main',
            'orbit_data_collector = drone_sim.orbit_data_collector:main',
            'orbit_detection = drone_sim.orbit_detection:main',
            'realtime_detector = drone_sim.realtime_detector:main',
            'imu_camera_collector = drone_sim.imu_camera_collector:main',
        ],
    },
)
