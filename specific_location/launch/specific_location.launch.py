from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    return LaunchDescription([
        Node(
            package='specific_location',
            executable='specific_location_executable',
            output='screen')
    ])