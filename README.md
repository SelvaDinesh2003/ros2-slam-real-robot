Real-World Differential Drive Robot (ROS 2 Jazzy)

A differential drive real robot with SLAM using ROS 2 Jazzy.
The robot supports teleoperation, mapping, and localization via slam_toolbox
, and is controlled via ESP32 + ROSArduinoBridge.

Prerequisites

ROS 2 Jazzy

slam_toolbox

Navigation2

twist_mux (multiplex multiple velocity sources)

twist_stamper (adds timestamp to twist messages)

teleop_twist_keyboard (for manual control)

sllidar_ros2 (for LiDAR integration)

diffdrive_arduino (ROS 2 driver for Arduino/ESP32)

serial (ROS 2 serial driver)

Install Required ROS 2 Packages
For Simulation
# Gazebo ROS bridge
sudo apt install ros-${ROS_DISTRO}-ros-gz

# ros2_control
sudo apt install ros-jazzy-ros2-control

# Navigation2
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# Twist Mux
sudo apt install ros-jazzy-twist-mux

For Real Robot (in addition to above, except ros-gz)
# Clone twist_stamper
cd ~/ros2_ws/src
git clone https://github.com/joshnewans/twist_stamper.git
cd ~/ros2_ws
colcon build --symlink-install --packages-select twist_stamper

# Clone diffdrive_arduino
cd ~/ros2_ws/src
git clone https://github.com/YJ0528/diffdrive_arduino.git
cd ~/ros2_ws
colcon build --symlink-install --packages-select diffdrive_arduino

# Clone serial
cd ~/ros2_ws/src
git clone https://github.com/joshnewans/serial.git
cd ~/ros2_ws
colcon build --symlink-install --packages-select serial

# Clone sllidar_ros2
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
cd ~/ros2_ws
colcon build --symlink-install --packages-select sllidar_ros2

Clone & Build Workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone git@github.com:<your-username>/real_world_slam.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

ESP32 Firmware

Upload firmware from firmware/ROSArduinoBridge using Arduino IDE or PlatformIO.

# Navigate to firmware folder
cd ~/ros2_ws/src/real_world_slam/firmware/ROSArduinoBridge

# Example using Arduino IDE
arduino --board esp32:esp32:esp32dev --port /dev/ttyUSB0 ROSArduinoBridge.ino


⚡ Ensure ESP32 is connected to motors, encoders, and IMU.

Running the Robot
source ~/ros2_ws/install/setup.bash
ros2 launch robot robot.launch.py

Teleoperation (Keyboard Control)

Open a new terminal and run:

source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard


Use your keyboard to control the robot.

Running SLAM

Open a new terminal and source the workspace:

source ~/ros2_ws/install/setup.bash

1. Online Mapping (recommended)
ros2 launch robot robot_slam.launch.py use_slam_option:=online_async_slam

2. Localization with Prebuilt Map
ros2 launch robot robot_slam.launch.py use_slam_option:=mapper_params_localization

Notes

Always source your workspace in each new terminal before running ROS 2 commands.

Use the use_slam_option argument to switch between mapping and localization.

Ensure the ESP32 firmware is uploaded and communicating with ROS 2 nodes.

All commands are inline using backticks for easy copy-paste.
