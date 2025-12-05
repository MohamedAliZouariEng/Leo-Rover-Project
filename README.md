## 🚜 Leo Rover Project 

This repository features an exciting robotics project using the Leo Rover platform, the ROS 2 Jazzy framework, and the Gazebo Harmonic simulator. The core of this project involves integrating advanced AI models, including one inspired by NASA's work specifically for plant detection, alongside custom implementations for autonomous navigation and text recognition.

-----

## 🛠️ Prerequisites and Setup

This project is developed and tested on **Ubuntu 24.04 (Noble Numbat)**. You will need to install ROS 2 Jazzy and several Python dependencies.

### 1\. ROS 2 Jazzy Installation

Follow the official documentation for installing ROS 2 Jazzy from Debian packages on Ubuntu:

  * 🔗 **Official Documentation:** [https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

After the installation is complete, set up your shell environment:

```bash
# Append the ROS 2 setup script to your .bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
# Source your .bashrc to apply the changes immediately
source ~/.bashrc
```

-----

### 2\. Python Environment and Dependencies

It is highly recommended to use a **Python virtual environment** to manage the required libraries and avoid conflicts with system packages.

```bash
# Navigate to the home directory
cd ~
# Create a new virtual environment named ros2_venv
python3 -m venv ros2_venv
# Activate the virtual environment
source ros2_venv/bin/activate

# Install core project dependencies
pip install pyyaml
pip install "numpy<2" # Specifically pin NumPy version if encountering issues with Torch
pip install torch torchvision torchaudio # PyTorch for AI models
pip install opencv-python # OpenCV for image processing
pip install transforms3d
pip3 install pytesseract # Optical Character Recognition (OCR) library
pip3 install imutils # Convenience functions for image processing

# Deactivate the environment once done installing
deactivate
```

-----

### 3\. Creating the ROS 2 Workspace and Cloning

We will create a ROS 2 workspace named `leo_ws` and clone the project repository into the `src` directory.

```bash
# Navigate to home and create the workspace structure
cd ~
mkdir -p leo_ws/src

# Clone the repository
git clone https://github.com/MohamedAliZouariEng/Leo-Rover-Project.git

# Move all package contents into the workspace source directory
cd Leo-Rover-Project
mv * ../leo_ws/src/

# Remove the now empty cloned directory
cd ../
rm -r Leo-Rover-Project
```

-----

### 4\. Building the Workspace and Environment Setup

Build the packages in your workspace using `colcon`.

```bash
# Navigate to the workspace root
cd ~/leo_ws/
# Build all packages
colcon build
```

Next, you need to set the Gazebo resource path so the simulator can find the Leo Rover model files. You also need to source the workspace setup file.

```bash
# Set the environment variable for Gazebo models
export GZ_SIM_RESOURCE_PATH=$HOME/leo_ws/src/leo_common-ros2/leo_description/urdf:$HOME/leo_ws/src:$HOME/leo_ws/src/leo_simulator-ros2/leo_gz_worlds/models
# Source the local workspace setup file
source install/setup.bash
```

*Note: You will need to run the `source install/setup.bash` command in every new terminal you use for ROS 2 commands.*

-----
## 📝 Important World Configuration Note

**Note:** The simulation world must be configured differently depending on the task being executed. Before running the project, please ensure the correct Gazebo world file is selected in the launch file.

1.  **For Tasks 1, 2, and 3 (Navigation, Plant Detection, Text Recognition):**

      * Edit the file: `~/leo_ws/src/leo_gz_bringup/launch/leo_gz.launch.py`
      * **Comment out** line 42 (which selects `space_terrain_flat_env.world`) and **uncomment** the line that uses `space_terrain_env_simple.world`.

    <!-- end list -->

    ```python
    default_value=os.path.join(pkg_project_worlds, "worlds", "space_terrain_env_simple.world"), # Line 42 - UNCOMMENT THIS
    # default_value=os.path.join(pkg_project_worlds, "worlds", "space_terrain_flat_env.world"), # Line 43 - COMMENT THIS
    
    ```

2.  **For Task 4 (Green Alien Plant Detection):**

      * Edit the file: `~/leo_ws/src/leo_gz_bringup/launch/leo_gz.launch.py`
      * **Comment out** line 43 (which selects `space_terrain_env_simple.world`) and **uncomment** line 42 to use `space_terrain_flat_env.world`.

    <!-- end list -->

    ```python
    # default_value=os.path.join(pkg_project_worlds, "worlds", "space_terrain_env_simple.world"), # Line 42 - COMMENT THIS
    default_value=os.path.join(pkg_project_worlds, "worlds", "space_terrain_flat_env.world"), # Line 43 - UNCOMMENT THIS
    
    ```

**After modifying the launch file, you must rebuild your workspace:**

```bash
cd ~/leo_ws/
colcon build
```
-----

## 🏎️ Running the Simulation

Launch the Leo Rover simulation in Gazebo.

```bash
ros2 launch leo_gz_bringup leo_gz.launch.py
```

A new window for the Gazebo simulator will open. **Remember to click the play button** (▶️ usually located at the bottom left) in the Gazebo GUI to start the simulation physics.

-----

## 🤖 Project Tasks

This project includes four distinct tasks demonstrating autonomous control, environment perception, and AI integration. For each task, you must open a **new terminal**, navigate to your workspace, and **source the setup file** (`source ~/leo_ws/install/setup.bash`).

### Task 1: Autonomous Lidar Navigation and Return-to-Origin

This node enables the Leo Rover to navigate autonomously, avoiding obstacles using its Lidar sensor. If the robot moves further than **5.0 meters** from the world's center, it will autonomously attempt to return to the origin.

  * **Goal:** Autonomous obstacle avoidance and geofencing/return-to-origin.
  * **Sensor:** Simulated Lidar.

<!-- end list -->

```bash
# In a new terminal:
cd ~/leo_ws/
source install/setup.bash
ros2 run leo_rover_tasks autonomous_exploration_executable
```

-----

### Task 2: Native Plant Detection (NASA AI Model)

This task uses a pre-trained AI model (inspired by NASA's work on planetary rovers for environmental perception) to detect and classify plants from the rover's camera feed.

  * **Model Location:** `~/leo_ws/src/basic_ros2_extra_files/plant_detector/best_plant_detector_model.pth`
  * **Goal:** Real-time plant detection in the camera stream.

<!-- end list -->

```bash
# In a new terminal:
cd ~/leo_ws/
source install/setup.bash
ros2 run leo_rover_tasks image_plant_detector_executable
```

-----

### Task 3: Text Recognition and Location

The Leo Rover is capable of identifying and recognizing text in its camera feed using the **EAST** (Efficient and Accurate Scene Text) deep learning model for text detection and the **Pytesseract** library for Optical Character Recognition (OCR).

#### 3.1 Basic Text Recognition

The basic executable processes the video stream to detect and recognize text.

  * **Model Location:** `~/leo_ws/src/basic_ros2_extra_files/text_detector/frozen_east_text_detection.pb`

<!-- end list -->

```bash
# In a new terminal:
cd ~/leo_ws/
source install/setup.bash
ros2 run leo_rover_tasks text_recog_executable
```

#### 3.2 Server/Client Specific Word Detection

These two nodes use a ROS 2 Service/Client architecture to specifically search for the word **"FOOD"** in the video stream and report its position.

  * **Service Server:** Waits for a request and handles the text detection logic.
    ```bash
    # In one terminal, run the server:
    ros2 run leo_rover_tasks text_recog_s_server_custom_executable
    ```
  * **Service Client:** Sends the request to trigger the search.
    ```bash
    # In another terminal, run the client:
    ros2 run leo_rover_tasks text_recog_s_client_custom_executable
    ```

-----


### Task 4: Green Alien Plant Detection and Environment Analysis

This task deploys a different NASA-inspired AI model to detect a **specific 'green alien plant'**. It uses two separate services for different functionalities: one to initiate the search and stop action, and another to display the color-filtered image for visual analysis.

  * **Model Location:** `~/leo_ws/src/basic_ros2_extra_files/plant_detector/best_alien_plant_detector_model.pth`
  * **Goal:** Search for the target plant, stop upon detection, and visualize green regions in the camera feed.

#### 4.1 Launch and Service Calls

1.  **Launch the Detector Node:** This node contains the logic and AI model.

    ```bash
    # In a new terminal:
    cd ~/leo_ws/
    source install/setup.bash
    ros2 launch basics_ros2_multithreading green_detector_node.launch.py
    ```

2.  **Call the Service to Start Detection and Stop:** This service call initiates the plant search process. **If the specified green alien plant is detected, the robot will stop its movement.**

    ```bash
    ros2 service call /detect_plants std_srvs/srv/Trigger "{}"
    ```

3.  **Call the Service to Visualize Green Areas:** This service call activates a separate visual analysis process. A new image window will pop up, displaying the camera feed where **all green areas are highlighted in white** for easy identification.

    ```bash
    ros2 service call /green_detector std_srvs/srv/Trigger "{}"
    ```


-----

## 🎮 Manual Teleoperation

You can manually control the Leo Rover using your keyboard with the following command:

```bash
# In a new terminal:
cd ~/leo_ws/
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

This is useful for manually driving the rover to test the different detection or navigation .

-----
