from setuptools import find_packages, setup
import os 
from glob import glob

from pathlib import Path

home_dir = Path.home()
venv_path = home_dir / 'ros2_venv' / 'bin' / 'python3'


package_name = 'leo_rover_tasks'


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
            'subscriber_obstacle_detector_executable = leo_rover_tasks.subscriber_obstacle_detector:main',
            'publish_leo_rover_move_executable = leo_rover_tasks.publish_leo_rover_move:main',
            'image_plant_detector_executable = leo_rover_tasks.plant_detector_node:main',
            'autonomous_exploration_executable = leo_rover_tasks.autonomous_exploration:main',
            'text_recog_executable = leo_rover_tasks.text_recog_node:main',
            'leo_rover_s_server_executable = leo_rover_tasks.leo_rover_status_s_server:main',
            'text_recog_s_server_executable = leo_rover_tasks.text_recog_s_server:main',
            'leo_rover_status_s_client_executable = leo_rover_tasks.leo_rover_status_s_client:main',
            'text_recog_s_client_executable = leo_rover_tasks.text_recog_s_client:main',
            'text_recog_s_client_custom_executable = leo_rover_tasks.text_recog_s_client_custom:main',
            'text_recog_s_server_custom_executable = leo_rover_tasks.text_recog_s_server_custom:main',
],
    },
    options={
        'build_scripts': {
            'executable': str(venv_path),
        },
    },
)
