import launch
import launch_ros.actions
from tracetools_launch.action import Trace
def generate_launch_description():
    # Trace
    trace = Trace(
        session_name='plant_detector'
    )
    # Nodes
    plant_detector_multithreading_callbackgroups_node = launch_ros.actions.Node(
        package='basics_ros2_multithreading',
        executable='plant_detector_multithreading_callbackgroups.py',
        arguments=[],
        output='screen',
    )
    return launch.LaunchDescription([
        trace,
        plant_detector_multithreading_callbackgroups_node
    ])  