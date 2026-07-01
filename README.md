# Rover Autonomy Heading and Moving to GPS

This repository contains a ROS2-based rover autonomy system for heading control and GPS-direction turning.

The current target platform is:

```text
Raspberry Pi 5
Ubuntu 24.04 Noble
ROS2 Jazzy
Arduino Mega
GY-271 / QMC5883L compass sensor
BTS7960 motor drivers
```

## Project Goal

The purpose of this project is to build a simple and reliable autonomous rover heading system.

At this stage, the rover does not perform full waypoint navigation. The current system focuses on:

```text
1. Reading compass heading from Arduino
2. Sending motor commands from Raspberry Pi over USB serial
3. Turning the rover to a target heading
4. Calculating target bearing from current GPS and target GPS coordinates
5. Turning the rover toward the target GPS direction
```

## System Architecture

```text
Raspberry Pi 5 / ROS2 Jazzy
    │
    ├── arduino_bridge_node
    │   ├── Reads serial telemetry from Arduino
    │   ├── Publishes compass heading to /compass/heading_deg
    │   ├── Publishes raw Arduino serial lines to /arduino/raw_line
    │   └── Subscribes to /cmd_drive and sends motor commands to Arduino
    │
    ├── heading_turn_node
    │   ├── Subscribes to /compass/heading_deg
    │   ├── Turns the rover to a target heading
    │   └── Publishes motor commands to /cmd_drive
    │
    └── gps_turn_node
        ├── Takes current GPS and target GPS as parameters
        ├── Calculates target bearing
        ├── Compares bearing with compass heading
        └── Publishes motor commands to /cmd_drive

Arduino Mega
    │
    ├── Reads GY-271 / QMC5883L compass sensor
    ├── Drives BTS7960 motor drivers
    ├── Sends MAG telemetry over USB serial
    └── Stops motors with watchdog if commands stop
```

## ROS2 Topics

```text
/compass/heading_deg
    Type: std_msgs/msg/Float32
    Publisher: arduino_bridge_node

/cmd_drive
    Type: std_msgs/msg/String
    Publisher: heading_turn_node or gps_turn_node
    Subscriber: arduino_bridge_node

/arduino/raw_line
    Type: std_msgs/msg/String
    Publisher: arduino_bridge_node

/arduino/status
    Type: std_msgs/msg/String
    Publisher: arduino_bridge_node

/navigation/target_heading_deg
    Type: std_msgs/msg/Float32
    Publisher: gps_turn_node
```

## Motor Command Protocol

The Raspberry Pi sends motor commands to Arduino through `/cmd_drive`.

Supported commands:

```text
MOTOR:STOP
MOTOR:FWD:<pwm>
MOTOR:BACK:<pwm>
MOTOR:LEFT:<pwm>
MOTOR:RIGHT:<pwm>
```

Examples:

```text
MOTOR:LEFT:80
MOTOR:RIGHT:80
MOTOR:STOP
```

Arduino receives these commands over USB serial and drives the BTS7960 motor drivers.

## Arduino Telemetry Format

Arduino sends compass telemetry in this format:

```text
MAG,time_ms,heading,rawX,rawY,rawZ,calX,calY,calZ,plane,offset,motor_mode,pwm
```

Example:

```text
MAG,123456,87.35,-850,120,-60,430.20,-180.50,12.40,XY,0.00,STOP,0
```

The `arduino_bridge_node` parses this line and publishes the heading value to:

```text
/compass/heading_deg
```

## Package Structure

```text
arc26_rover_bringup/
├── arc26_rover_bringup/
│   ├── __init__.py
│   ├── gps_math.py
│   ├── arduino_bridge_node.py
│   ├── heading_turn_node.py
│   └── gps_turn_node.py
│
├── resource/
│   └── arc26_rover_bringup
│
├── test/
│
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
└── .gitignore
```

## Main Files

```text
gps_math.py
    Contains heading normalization, angle error calculation, and GPS bearing calculation.

arduino_bridge_node.py
    ROS2 serial bridge between Raspberry Pi and Arduino.

heading_turn_node.py
    Turns the rover to a fixed target heading.

gps_turn_node.py
    Calculates bearing from current GPS to target GPS and turns the rover toward that direction.
```

## Requirements

Install required packages:

```bash
sudo apt update
sudo apt install -y python3-serial python3-colcon-common-extensions
```

Add the user to the serial group:

```bash
sudo usermod -a -G dialout $USER
sudo reboot
```

## Build

Go to the ROS2 workspace:

```bash
cd ~/arc26_ros2_ws
```

Source ROS2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
```

Build the workspace:

```bash
colcon build --symlink-install
```

Source the workspace:

```bash
source install/setup.bash
```

Optional: add sources to `.bashrc`:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/arc26_ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Check Arduino Port

Connect Arduino Mega to Raspberry Pi over USB.

Check the port:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

Usually Arduino Mega appears as:

```text
/dev/ttyACM0
```

If it appears as `/dev/ttyUSB0`, use that port when running the bridge node.

## Run Arduino Bridge Node

## Run Arduino Bridge Node

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source ~/arc26_ros2_ws/install/setup.bash

ros2 run arc26_rover_bringup arduino_bridge_node \
  --ros-args \
  -p port:=/dev/ttyACM0 \
  -p baud:=115200 \
  -p plane:=XY \
  -p heading_offset_deg:=-53.5

This node should stay running.

## Monitor Compass Heading

Terminal 2:

```bash
source ~/arc26_ros2_ws/install/setup.bash
ros2 topic echo /compass/heading_deg
```

When the rover is rotated by hand, the heading value should change.

## Monitor Raw Arduino Serial Data

```bash
ros2 topic echo /arduino/raw_line
```

## Manual Motor Command Test

Turn left:

```bash
ros2 topic pub /cmd_drive std_msgs/msg/String "{data: 'MOTOR:LEFT:80'}" -r 5
```

Stop:

```bash
ros2 topic pub /cmd_drive std_msgs/msg/String "{data: 'MOTOR:STOP'}" -1
```

Turn right:

```bash
ros2 topic pub /cmd_drive std_msgs/msg/String "{data: 'MOTOR:RIGHT:80'}" -r 5
```

Stop again:

```bash
ros2 topic pub /cmd_drive std_msgs/msg/String "{data: 'MOTOR:STOP'}" -1
```

## Turn Rover to Target Heading

Keep `arduino_bridge_node` running in another terminal.

Example: turn rover to 90 degrees:

```bash
source ~/arc26_ros2_ws/install/setup.bash

ros2 run arc26_rover_bringup heading_turn_node \
  --ros-args \
  -p target_heading:=90.0
```

Other examples:

```bash
ros2 run arc26_rover_bringup heading_turn_node \
  --ros-args \
  -p target_heading:=180.0
```

```bash
ros2 run arc26_rover_bringup heading_turn_node \
  --ros-args \
  -p target_heading:=270.0
```

If the rover turns in the wrong direction:

```bash
ros2 run arc26_rover_bringup heading_turn_node \
  --ros-args \
  -p target_heading:=90.0 \
  -p invert_turn_direction:=true
```

## Turn Rover Toward Target GPS Direction

Keep `arduino_bridge_node` running.

Run:

```bash
source ~/arc26_ros2_ws/install/setup.bash

ros2 run arc26_rover_bringup gps_turn_node \
  --ros-args \
  -p cur_lat:=39.815988 \
  -p cur_lon:=30.528493 \
  -p tgt_lat:=39.816100 \
  -p tgt_lon:=30.529000
```

The node calculates the bearing from the current GPS coordinate to the target GPS coordinate, compares it with the current compass heading, and sends motor commands until the rover faces the target direction.

## Useful Debug Commands

List ROS2 topics:

```bash
ros2 topic list
```

Echo heading:

```bash
ros2 topic echo /compass/heading_deg
```

Echo target GPS bearing:

```bash
ros2 topic echo /navigation/target_heading_deg
```

Echo motor commands:

```bash
ros2 topic echo /cmd_drive
```

Echo Arduino status:

```bash
ros2 topic echo /arduino/status
```

Echo raw Arduino serial lines:

```bash
ros2 topic echo /arduino/raw_line
```

## Safety

The Arduino firmware includes a watchdog. If the Raspberry Pi or ROS2 node stops sending motor commands, Arduino automatically stops the motors after a short timeout.

This prevents the rover from continuing to move indefinitely after a software crash, USB disconnect, or ROS2 node failure.

## Current Development Status

Working:

```text
Arduino serial bridge
Compass heading topic
Manual motor command through ROS2 topic
Target heading turn
GPS bearing calculation
GPS-direction turning
```

Next planned steps:

```text
1. Add real GPS reader node
2. Publish current GPS as a ROS2 topic
3. Publish target GPS as a ROS2 topic
4. Replace string motor commands with a custom DriveCommand message
5. Add launch files
6. Add waypoint navigation
7. Add forward movement after heading alignment
8. Add logging and rosbag support
```
