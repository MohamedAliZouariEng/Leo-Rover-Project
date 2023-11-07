# Prototypes

## ros2_tracing
We need this tool to be able to visualise in someway the callback in ROS2.

Based on the roscon 2021 video:
Code example: https://github.com/christophebedard/ros-world-2021-demo.git

Lets try:

### Setup rso2_tracing

These are teh command sto insrtall and test it


```
# For the moment I didn't install it

sudo apt-get update
sudo apt-get install lttng-tools liblttng-ust-dev
sudo apt-get install python3-babeltrace python3-lttng

# Optional 
sudo apt-get install lttng-modules-dkms



cd ros2_ws/src
git clone -b humble https://gitlab.com/ros-tracing/ros2_tracing.git
cd ..
colcon build --packages-up-to tracetools

# Message when co,ìling:
# --- stderr: tracetools                              
# LTTng found: tracing enabled


source ./install/setup.bash
ros2 run tracetools status
# Output: Tracing enabled

# If we want to disable, we have to rebuild:
# colcon build --cmake-args " -DTRACETOOLS_DISABLED=ON"

# Tracing disabled through configuration
# [ros2run]: Process exited with failure 1
## TO REINSTATE
# colcon build --cmake-args " -DTRACETOOLS_DISABLED=OFF"



cd ros2_ws/src
git clone https://gitlab.com/ros-tracing/tracetools_analysis.git
git clone https://github.com/christophebedard/ros-world-2021-demo.git
cd ..
pip3 install notebook bokeh
colcon build --packages-up-to ros_world_2021_demo

```

Lets test the example:

```
source install/setup.bash
ros2 launch tracetools_launch example.launch.py

source install/setup.bash
ros2 launch ros_world_2021_demo demo.launch.py


# Analyse
# RAw data:
babeltrace ~/.ros/tracing/ros-world-2021-demo/

# We need to process this data:
pip3 install notebook bokeh
jupyter notebook src/ros-world-2021-demo/ros_world_2021_demo/analysis/demo.ipynb

```

### CUSTOM MADE EXAMPLES STEP BY STEP


