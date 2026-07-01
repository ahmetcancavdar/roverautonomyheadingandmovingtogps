from setuptools import find_packages, setup

package_name = 'arc26_rover_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='pi8gb',
    maintainer_email='ahmetcan5841@gmail.com',
    description='ARC26 rover Raspberry Pi 5 ROS2 Jazzy autonomy bridge and heading control.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_bridge_node = arc26_rover_bringup.arduino_bridge_node:main',
            'heading_turn_node = arc26_rover_bringup.heading_turn_node:main',
            'gps_turn_node = arc26_rover_bringup.gps_turn_node:main',
        ],
    },
)
