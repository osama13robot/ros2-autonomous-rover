from setuptools import find_packages, setup

package_name = 'rover_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Osama',
    maintainer_email='osama.mechatronics.13@gmail.com',
    description='Teleop, mode manager, frontier explorer and sim adapter for the rover.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = rover_control.teleop_keyboard:main',
            'mode_manager = rover_control.mode_manager:main',
            'frontier_explorer = rover_control.frontier_explorer:main',
            'sim_sensor_adapter = rover_control.sim_sensor_adapter:main',
            'waypoint_patrol = rover_control.waypoint_patrol:main',
        ],
    },
)
