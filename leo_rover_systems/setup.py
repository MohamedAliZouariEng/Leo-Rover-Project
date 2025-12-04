from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'leo_rover_systems'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu24',
    maintainer_email='ubuntu24@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'heartbeat_executable_1 = leo_rover_systems.heartbeat:main1',
            'heartbeat_executable_shutdown = leo_rover_systems.heartbeat:main_shutdown',
            'heartbeat_executable_2 = leo_rover_systems.heartbeat:main2',
            'temperaturemonitor_executable = leo_rover_systems.temperature_monitor:start_monitor'
        ],
    },
)
