import launch
import launch_ros.actions
from tracetools_launch.action import Trace



def generate_launch_description():
    # Trace     
    trace = Trace(
    session_name='green_detector'
    )
    # Nodes
    green_detector_node_node = launch_ros.actions.Node(
        package='basics_ros2_multithreading',
        executable='green_detector_node.py',
        arguments=[],
        output='screen',
    )
    return launch.LaunchDescription([
        trace,
        green_detector_node_node
    ])  