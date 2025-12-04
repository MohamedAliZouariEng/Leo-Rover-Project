from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    return LaunchDescription([
        Node(
            package='services_turn',
            executable='turn_s_client_executable',
            output='screen')
    ])