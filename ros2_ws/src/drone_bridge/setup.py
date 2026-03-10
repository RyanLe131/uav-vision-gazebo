from setuptools import find_packages, setup

package_name = 'drone_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'drone_bridge/gz_bridge.launch.py',
            'drone_bridge/camera_view.launch.py',
            'drone_bridge/opencv_viewer.launch.py'
        ]),
        ('share/' + package_name + '/scripts', [
            'scripts/camera_viewer.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryanleai',
    maintainer_email='ryanleai@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
