from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'my_action_client'

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
    maintainer_email='mohamedalizouari.ai@mail.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example72 = my_action_client.action_client:main',
            'example75 = my_action_client.custom_action_client:main'
        ],
    },
)
