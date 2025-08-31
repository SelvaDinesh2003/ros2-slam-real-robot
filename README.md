# Real-World Differential Drive Robot (ROS 2 Jazzy)

A differential drive real robot with SLAM using ROS 2 Jazzy.
The robot supports teleoperation, mapping, and localization via slam_toolbox
, and is controlled via ESP32 + ROSArduinoBridge.

## Prerequisites

ROS 2 Jazzy

slam_toolbox

twist_mux (multiplex multiple velocity sources)

twist_stamper (adds timestamp to twist messages)

teleop_twist_keyboard (for manual control)

sllidar_ros2 (for LiDAR integration)

diffdrive_arduino (ROS 2 driver for Arduino/ESP32)

serial (ROS 2 serial driver)

## Install Required ROS 2 Packages

### Gazebo ROS bridge
`sudo apt install ros-${ROS_DISTRO}-ros-gz`

### ros2_control
`sudo apt install ros-jazzy-ros2-control`

### Twist Mux
`sudo apt install ros-jazzy-twist-mux`

### Clone twist_stamper(source)
`cd ~/ros2_ws/src`

`git clone https://github.com/joshnewans/twist_stamper.git`

`cd ~/ros2_ws`

`colcon build --symlink-install --packages-select twist_stamper`

### Clone diffdrive_arduino (source)
`cd ~/ros2_ws/src`

`git clone https://github.com/YJ0528/diffdrive_arduino.git`

`cd ~/ros2_ws`

`colcon build --symlink-install --packages-select diffdrive_arduino`

### Clone serial (source)
`cd ~/ros2_ws/src`

`git clone https://github.com/joshnewans/serial.git`

`cd ~/ros2_ws`

`colcon build --symlink-install --packages-select serial`

# Clone sllidar_ros2 (source)
`cd ~/ros2_ws/src`

`git clone https://github.com/Slamtec/sllidar_ros2.git`

`cd ~/ros2_ws`

`colcon build --symlink-install --packages-select sllidar_ros2`

## Clone & Build Workspace
`mkdir -p ~/ros2_ws/src`

`cd ~/ros2_ws/src`

`git clone git@github.com:<your-username>/real_world_slam.git`

`cd ~/ros2_ws`

`colcon build --symlink-install`

`source install/setup.bash`

## ESP32 Firmware
Upload firmware from firmware/ROSArduinoBridge using Arduino IDE or PlatformIO to Esp32 .

## Electronics Used
ESP32 – motor + encoder interface, running ROSArduinoBridge firmware

Raspberry Pi 5 – main ROS 2 Jazzy computer

Slamtec RPLIDAR C1 – 2D LiDAR for SLAM & localization

Two DC Motors with Encoders – differential drive setup

Motor Driver – L298N / TB6612FNG or similar

3 × Lithium Batteries – for motor power

Power Banks with PD (Power Delivery) – powering Raspberry Pi 5 and peripherals

## Connections

ESP32 → Motor Driver

GPIO pins → IN1/IN2 (left motor)

GPIO pins → IN3/IN4 (right motor)

PWM pins → ENA/ENB (motor speed control)

Encoders → ESP32

Encoder A/B channels → GPIO interrupt pins

ESP32 → Raspberry Pi 5

Serial connection (USB cable or UART)

RPLIDAR C1 → Raspberry Pi 5

For reference, see the circuit diagram in the repository:[View Circuit Diagram](https://github.com/SelvaDinesh2003/ros2-slam-real-robot/blob/main/docs/circuit.png)
## Remote Machine Setup (SSH + ROS 2 Networking)
The Raspberry Pi 5 runs the robot stack, while your laptop/server machine is used for teleoperation, SLAM, and visualization.

### 1. Establish SSH Connection (from Laptop → Pi)
Run on laptop/server:
`ssh <username>@<raspberry_pi_ip>`

Example:
`ssh pi@192.168.1.25`

### 2. Configure ROS_DOMAIN_ID (both machines)
ROS 2 nodes discover each other only if they share the same domain ID (default: 0, allowed: 0–101).
Run on both Raspberry Pi and Laptop:
`export ROS_DOMAIN_ID=1`

Make it permanent on Raspberry Pi:
`echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc`

## Running the Robot

`source ~/ros2_ws/install/setup.bash`

`ros2 launch robot robot.launch.py`

## Teleoperation (Keyboard Control)
Use your keyboard to control the robot.
Open a new terminal and run:

`source ~/ros2_ws/install/setup.bash`

`ros2 run teleop_twist_keyboard teleop_twist_keyboard`

## Running SLAM
## 1. Online Mapping

`source ~/ros2_ws/install/setup.bash`

`ros2 launch robot robot_slam.launch.py use_slam_option:=online_async_slam`

## 2. Localization with Prebuilt Map

`ros2 launch robot robot_slam.launch.py use_slam_option:=mapper_params_localization`

Notes

Always source your workspace in each new terminal before running ROS 2 commands.

Use the use_slam_option argument to switch between mapping and localization.

Ensure the ESP32 firmware is uploaded and communicating with ROS 2 nodes.
