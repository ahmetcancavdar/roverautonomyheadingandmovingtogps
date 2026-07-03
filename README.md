# Rover Autonomy Heading and Moving to GPS

This repository contains a ROS2-based rover autonomy system for heading control, RTK GPS reading, GPS bearing calculation, and waypoint navigation.

The current target platform is:

```text
Raspberry Pi 5
Ubuntu 24.04 Noble
ROS2 Jazzy
Arduino Mega
LC29HEA RTK GPS
3DR SiK telemetry radio
GY-271 / QMC5883L compass sensor
BTS7960 motor drivers
```

---

## Project Goal

The purpose of this project is to build a simple and reliable autonomous rover navigation system.

The current system can:

```text
1. Read compass heading from Arduino.
2. Send motor commands from Raspberry Pi over USB serial.
3. Turn the rover to a target heading.
4. Calculate target bearing from current GPS and target GPS coordinates.
5. Turn the rover toward the target GPS direction.
6. Read live RTK GPS data from LC29HEA.
7. Receive RTCM corrections through a 3DR SiK telemetry radio.
8. Move toward a target GPS waypoint.
9. Continuously refresh heading and GPS distance while moving.
10. Stop when the rover enters the target radius.
```

The waypoint navigation system always uses the best available GPS data. RTK_FIXED is not required for movement anymore.

```text
RTK_FIXED available      -> stop radius = 0.30 m
RTK_FIXED not available  -> stop radius = 0.60 m
No GPS data              -> stop motors
No heading data          -> stop motors
```

---

## Current Tested Configuration

### Hardware

```text
Raspberry Pi 5
Ubuntu 24.04 Noble
ROS2 Jazzy
Arduino Mega
LC29HEA RTK GPS
3DR SiK telemetry radio
GY-271 / QMC5883L compass
BTS7960 motor drivers
```

### Serial Ports on Rover Raspberry Pi 5

```text
/dev/ttyUSB1  -> Rover RTK GPS
/dev/ttyUSB0  -> RF / 3DR SiK telemetry radio
/dev/ttyACM0  -> Arduino Mega
```

### Tuned Parameters

```text
heading_offset_deg = -53.5

heading_turn_node heading_tolerance_deg = 2.75
gps_turn_node heading_tolerance_deg = 2.75

waypoint_nav_node heading_tolerance_deg = 5.0
waypoint_nav_node reacquire_heading_error_deg = 10.0

fixed_goal_radius_m = 0.30
non_fixed_goal_radius_m = 0.60

turn_pwm_slow = 45
turn_pwm_fast = 75
forward_pwm = 60
slow_turn_threshold_deg = 30.0
command_interval_sec = 0.25
```

Important notes:

```text
heading_turn_node and gps_turn_node keep 2.75 degrees for precise static turning.

waypoint_nav_node uses 5.0 / 10.0 hysteresis because it drives while GPS and compass values are changing.

RTK_FIXED is not required for movement.
If GPS is valid but not RTK_FIXED, the rover continues with 0.60 m target radius.
If RTK_FIXED is available, the rover uses 0.30 m target radius.
```

---

## System Architecture

```text
Base Computer / Base Raspberry
    │
    ├── basertkgps_5dk_survey.py
    │   ├── Configures LC29HEA as RTK base
    │   ├── Performs survey-in or fixed ECEF base setup
    │   ├── Extracts valid RTCM3 frames
    │   └── Sends RTCM corrections to 3DR SiK radio

Raspberry Pi 5 / ROS2 Jazzy
    │
    ├── gps_rtk_node
    │   ├── Reads RTCM data from rover 3DR SiK radio
    │   ├── Writes RTCM frames to rover LC29HEA GPS
    │   ├── Reads GGA NMEA data from rover GPS
    │   ├── Publishes /gps/fix
    │   ├── Publishes /gps/rtk_status
    │   └── Publishes /gps/rtcm_status
    │
    ├── arduino_bridge_node
    │   ├── Reads serial telemetry from Arduino
    │   ├── Publishes compass heading to /compass/heading_deg
    │   ├── Publishes raw Arduino serial lines to /arduino/raw_line
    │   ├── Publishes Arduino status to /arduino/status
    │   └── Subscribes to /cmd_drive and sends motor commands to Arduino
    │
    ├── heading_turn_node
    │   ├── Subscribes to /compass/heading_deg
    │   ├── Turns the rover to a fixed target heading
    │   └── Publishes motor commands to /cmd_drive
    │
    ├── gps_turn_node
    │   ├── Takes current GPS and target GPS as parameters
    │   ├── Calculates target bearing
    │   ├── Compares bearing with compass heading
    │   └── Publishes motor commands to /cmd_drive
    │
    └── waypoint_nav_node
        ├── Subscribes to /gps/fix
        ├── Subscribes to /gps/rtk_status
        ├── Subscribes to /compass/heading_deg
        ├── Calculates distance to target GPS
        ├── Calculates target bearing continuously
        ├── Turns toward target when heading error is large
        ├── Moves forward when heading error is acceptable
        ├── Uses hysteresis to reduce oscillation
        ├── Uses 0.30 m radius if RTK_FIXED is active
        ├── Uses 0.60 m radius if RTK_FIXED is not active but GPS is valid
        └── Stops if GPS or heading data is lost

Arduino Mega
    │
    ├── Reads GY-271 / QMC5883L compass sensor
    ├── Drives BTS7960 motor drivers
    ├── Receives motor commands over USB serial
    ├── Sends MAG telemetry over USB serial
    └── Stops motors with watchdog if commands stop
```

---

## ROS2 Topics

```text
/compass/heading_deg
    Type: std_msgs/msg/Float32
    Publisher: arduino_bridge_node

/cmd_drive
    Type: std_msgs/msg/String
    Publisher: heading_turn_node, gps_turn_node, or waypoint_nav_node
    Subscriber: arduino_bridge_node

/arduino/raw_line
    Type: std_msgs/msg/String
    Publisher: arduino_bridge_node

/arduino/status
    Type: std_msgs/msg/String
    Publisher: arduino_bridge_node

/gps/fix
    Type: sensor_msgs/msg/NavSatFix
    Publisher: gps_rtk_node

/gps/rtk_status
    Type: std_msgs/msg/String
    Publisher: gps_rtk_node

/gps/rtcm_status
    Type: std_msgs/msg/String
    Publisher: gps_rtk_node

/navigation/target_heading_deg
    Type: std_msgs/msg/Float32
    Publisher: gps_turn_node or waypoint_nav_node

/navigation/distance_to_target_m
    Type: std_msgs/msg/Float32
    Publisher: waypoint_nav_node

/navigation/status
    Type: std_msgs/msg/String
    Publisher: waypoint_nav_node
```

---

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
MOTOR:FWD:60
MOTOR:STOP
```

Arduino receives these commands over USB serial and drives the BTS7960 motor drivers.

---

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

---

## RTK GPS GGA Quality Codes

The rover RTK GPS node reads GGA NMEA sentences and interprets fix quality as:

```text
q=0 -> INVALID
q=1 -> SPS
q=2 -> DGPS
q=4 -> RTK_FIXED
q=5 -> RTK_FLOAT
q=6 -> DEAD_RECKONING
```

Waypoint behavior:

```text
q=4 RTK_FIXED                 -> move and stop inside 0.30 m
q=5 RTK_FLOAT                 -> move and stop inside 0.60 m
q=1 SPS                       -> move and stop inside 0.60 m
No RTCM but valid GPS position -> move and stop inside 0.60 m
No valid GPS position          -> stop motors
GPS timeout                    -> stop motors
No heading data                -> stop motors
Heading timeout                -> stop motors
```

---

## Package Structure

```text
arc26_rover_bringup/
├── arc26_rover_bringup/
│   ├── __init__.py
│   ├── gps_math.py
│   ├── arduino_bridge_node.py
│   ├── heading_turn_node.py
│   ├── gps_turn_node.py
│   ├── gps_rtk_node.py
│   └── waypoint_nav_node.py
│
├── resource/
│   └── arc26_rover_bringup
│
├── test/
│
├── tools/
│   ├── basertkgps_5dk_survey.py
│   └── roverrtkradio_ayni.py
│
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
└── .gitignore
```

---

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

gps_rtk_node.py
    Reads RTCM corrections from the RF radio, writes them to the rover GPS, reads GGA data, and publishes ROS2 GPS topics.

waypoint_nav_node.py
    Uses live GPS and compass heading to move toward a target GPS coordinate.

tools/basertkgps_5dk_survey.py
    Runs on the base computer. Configures base GPS and sends RTCM corrections through the RF radio.

tools/roverrtkradio_ayni.py
    Older standalone rover RTK test script. The ROS2 system uses gps_rtk_node.py instead.
```

---

## Requirements

Install required packages:

```bash
sudo apt update
sudo apt install -y python3-serial python3-colcon-common-extensions ros-jazzy-sensor-msgs
```

Add the user to the serial group:

```bash
sudo usermod -a -G dialout $USER
sudo reboot
```

Optional but recommended for USB serial stability:

```bash
sudo systemctl stop ModemManager 2>/dev/null
sudo systemctl disable ModemManager 2>/dev/null
```

---

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

---

## Check Serial Ports

Connect Arduino Mega, rover RTK GPS, and RF radio to Raspberry Pi.

Check ports:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

Expected rover-side ports:

```text
/dev/ttyUSB1  -> Rover RTK GPS
/dev/ttyUSB0  -> RF / 3DR SiK telemetry radio
/dev/ttyACM0  -> Arduino Mega
```

More detailed check:

```bash
ls -l /dev/serial/by-id/ 2>/dev/null
```

---

## Base RTK Startup

Run this on the base computer or base-side Raspberry.

Base GPS and base RF radio must be connected to the base computer.

```bash
python3 tools/basertkgps_5dk_survey.py
```

The base script performs survey-in and then starts RTCM transmission.

Wait until survey-in is completed and RTCM transmission starts:

```text
[OK] Survey-in bekleme süresi tamamlandı. RTCM aktarımı başlatılıyor.
[BASE TX] ... byte/s | ... frame/s | msg: 1005:1, 1077:1, 1087:1, ...
```

If the base keeps printing:

```text
[BASE TX] RTCM yok. Base GPS RTCM üretiyor mu kontrol et.
```

then the base GPS is not producing RTCM yet, or the base GPS/radio ports are wrong.

RTCM is useful for high accuracy. However, waypoint navigation no longer stops only because RTK_FIXED is missing. If rover GPS still provides a valid position, waypoint navigation continues with the non-fixed radius.

---

## Rover RTK GPS Node

Terminal 1 on Raspberry Pi 5:

```bash
source /opt/ros/jazzy/setup.bash
source ~/arc26_ros2_ws/install/setup.bash

ros2 run arc26_rover_bringup gps_rtk_node \
  --ros-args \
  -p gps_port:=/dev/ttyUSB1 \
  -p radio_port:=/dev/ttyUSB0 \
  -p gps_baud:=460800 \
  -p radio_baud:=57600
```

This node should stay running.

Check RTK status:

```bash
ros2 topic echo /gps/rtk_status
```

Examples:

```text
q=4,fix=RTK_FIXED
q=5,fix=RTK_FLOAT
q=1,fix=SPS
```

Check RTCM status:

```bash
ros2 topic echo /gps/rtcm_status
```

Good RTCM status example:

```text
rtcm_bytes_per_sec=...,rtcm_frames_per_sec=...,msg=1005:1,1077:1,1087:1,...
```

If it shows:

```text
msg=NO_RTCM
```

then base correction data is not reaching the rover radio.

Check GPS fix topic:

```bash
ros2 topic echo /gps/fix
```

Expected output includes:

```text
latitude: 39.816...
longitude: 30.528...
altitude: ...
```

---

## Arduino Bridge Node

Terminal 2 on Raspberry Pi 5:

```bash
source /opt/ros/jazzy/setup.bash
source ~/arc26_ros2_ws/install/setup.bash

ros2 run arc26_rover_bringup arduino_bridge_node \
  --ros-args \
  -p port:=/dev/ttyACM0 \
  -p baud:=115200 \
  -p plane:=XY \
  -p heading_offset_deg:=-53.5
```

This node should stay running.

The current tested compass offset is:

```text
heading_offset_deg = -53.5
```

If the compass sensor is moved, rotated, mounted in a different place, or if the magnetic environment changes, this offset must be recalibrated.

---

## Monitor Compass Heading

Terminal 3:

```bash
source /opt/ros/jazzy/setup.bash
source ~/arc26_ros2_ws/install/setup.bash

ros2 topic echo /compass/heading_deg
```

When the rover is rotated by hand, the heading value should change.

---

## Monitor Raw Arduino Serial Data

```bash
ros2 topic echo /arduino/raw_line
```

---

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

---

## Turn Rover to Target Heading

Keep `arduino_bridge_node` running in another terminal.

Example: turn rover to 90 degrees:

```bash
source /opt/ros/jazzy/setup.bash
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

Default heading tolerance for this node:

```text
heading_tolerance_deg = 2.75
```

---

## Turn Rover Toward Target GPS Direction

Keep `arduino_bridge_node` running.

Run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/arc26_ros2_ws/install/setup.bash

ros2 run arc26_rover_bringup gps_turn_node \
  --ros-args \
  -p cur_lat:=39.815988 \
  -p cur_lon:=30.528493 \
  -p tgt_lat:=39.816100 \
  -p tgt_lon:=30.529000
```

The node calculates the bearing from the current GPS coordinate to the target GPS coordinate, compares it with the current compass heading, and sends motor commands until the rover faces the target direction.

Default heading tolerance for this node:

```text
heading_tolerance_deg = 2.75
```

---

## Waypoint Navigation

`waypoint_nav_node` performs live GPS-based movement.

It uses:

```text
/gps/fix
/compass/heading_deg
/gps/rtk_status
```

The node continuously calculates:

```text
1. Current rover GPS position
2. Target GPS position
3. Distance to target
4. Bearing to target
5. Heading error
6. Required motor command
7. Active goal radius
```

The rover does not stop only because RTK_FIXED is missing.

RTK status only changes the target stop radius:

```text
RTK_FIXED / q=4        -> fixed_goal_radius_m = 0.30 m
RTK_FLOAT / q=5        -> non_fixed_goal_radius_m = 0.60 m
SPS / q=1              -> non_fixed_goal_radius_m = 0.60 m
No RTCM but GPS valid  -> non_fixed_goal_radius_m = 0.60 m
No GPS data            -> STOP
GPS timeout            -> STOP
No heading data        -> STOP
Heading timeout        -> STOP
```

This means:

```text
If RTK_FIXED is available:
    The rover stops inside a 30 cm radius.

If RTK_FIXED is not available but GPS position is valid:
    The rover still moves and stops inside a 60 cm radius.

If GPS position is not available:
    The rover stops the motors.
```

Run waypoint navigation:

```bash
source /opt/ros/jazzy/setup.bash
source ~/arc26_ros2_ws/install/setup.bash

ros2 run arc26_rover_bringup waypoint_nav_node \
  --ros-args \
  -p target_lat:=39.816056 \
  -p target_lon:=30.528806 \
  -p fixed_goal_radius_m:=0.30 \
  -p non_fixed_goal_radius_m:=0.60 \
  -p heading_tolerance_deg:=5.0 \
  -p reacquire_heading_error_deg:=10.0 \
  -p forward_pwm:=60 \
  -p turn_pwm_slow:=45 \
  -p turn_pwm_fast:=75 \
  -p slow_turn_threshold_deg:=30.0 \
  -p command_interval_sec:=0.25
```

Expected behavior:

```text
1. Rover reads live GPS position from /gps/fix.
2. Rover reads compass heading from /compass/heading_deg.
3. Rover calculates distance to the target GPS coordinate.
4. Rover calculates bearing to the target.
5. Rover turns toward the target if heading error is large.
6. Rover moves forward if heading error is acceptable.
7. Rover keeps refreshing GPS distance and target bearing while moving.
8. If RTK_FIXED is active, it stops inside 0.30 m.
9. If RTK_FIXED is not active but GPS is valid, it stops inside 0.60 m.
10. If GPS or heading data is lost, it stops.
```

---

## Heading Hysteresis in Waypoint Navigation

Waypoint navigation uses hysteresis to reduce rapid left-right oscillation.

The logic is:

```text
If nav_mode is TURN:
    switch to FORWARD only when abs_error <= heading_tolerance_deg

If nav_mode is FORWARD:
    switch back to TURN only when abs_error >= reacquire_heading_error_deg
```

Current tested values:

```text
heading_tolerance_deg = 5.0
reacquire_heading_error_deg = 10.0
```

This means:

```text
The rover starts by turning toward the target.
When heading error becomes 5 degrees or less, it moves forward.
While moving forward, it does not re-enter turn mode for small heading changes.
It only starts turning again if the error grows to 10 degrees or more.
```

This prevents the rover from rapidly switching between:

```text
LEFT -> RIGHT -> LEFT -> RIGHT
```

or:

```text
TURN -> FORWARD -> TURN -> FORWARD
```

near the heading threshold.

---

## Example Target Coordinate

Current test target:

```text
39°48'57.8"N 30°31'43.7"E
```

Decimal:

```text
target_lat = 39.816056
target_lon = 30.528806
```

---

## Useful Debug Commands

List ROS2 nodes:

```bash
ros2 node list
```

List ROS2 topics:

```bash
ros2 topic list
```

Echo heading:

```bash
ros2 topic echo /compass/heading_deg
```

Echo GPS fix:

```bash
ros2 topic echo /gps/fix
```

Echo RTK status:

```bash
ros2 topic echo /gps/rtk_status
```

Echo RTCM status:

```bash
ros2 topic echo /gps/rtcm_status
```

Echo target GPS bearing:

```bash
ros2 topic echo /navigation/target_heading_deg
```

Echo distance to target:

```bash
ros2 topic echo /navigation/distance_to_target_m
```

Echo waypoint navigation status:

```bash
ros2 topic echo /navigation/status
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

---

## Emergency Stop

Use this command immediately if the rover behaves unexpectedly:

```bash
source /opt/ros/jazzy/setup.bash
source ~/arc26_ros2_ws/install/setup.bash

ros2 topic pub /cmd_drive std_msgs/msg/String "{data: 'MOTOR:STOP'}" -1
```

You can also stop active ROS2 nodes with:

```bash
pkill -f waypoint_nav_node
pkill -f heading_turn_node
pkill -f gps_turn_node
```

---

## Safety

The Arduino firmware includes a watchdog. If the Raspberry Pi or ROS2 node stops sending motor commands, Arduino automatically stops the motors after a short timeout.

This prevents the rover from continuing to move indefinitely after:

```text
Software crash
USB disconnect
ROS2 node failure
Command timeout
```

Still, an emergency stop command should always be ready during field tests.

---

## Current Development Status

Working:

```text
Arduino serial bridge
Compass heading topic
Manual motor command through ROS2 topic
Target heading turn
GPS bearing calculation
GPS-direction turning
RTK GPS GGA parsing
RTCM radio-to-GPS forwarding
/gps/fix publishing
/gps/rtk_status publishing
/gps/rtcm_status publishing
Live waypoint navigation
Distance-to-target calculation
Heading hysteresis for stable movement
Dynamic target radius based on RTK status
RTK_FIXED and non-fixed GPS fallback behavior
```

Current tuned values:

```text
Arduino bridge heading offset: -53.5
Static heading turn tolerance: 2.75 degrees
GPS direction turn tolerance: 2.75 degrees
Waypoint forward threshold: 5.0 degrees
Waypoint reacquire threshold: 10.0 degrees
RTK_FIXED goal radius: 0.30 m
Non-fixed GPS goal radius: 0.60 m
```

Next planned steps:

```text
1. Improve launch files for one-command startup.
2. Add waypoint list support.
3. Add automatic target GPS topic/service.
4. Add rosbag logging.
5. Replace string motor commands with a custom DriveCommand message.
6. Add better GPS/heading filtering.
7. Add low-speed final approach mode near target.
8. Add web or GUI monitoring panel.
```

---

## Recommended Startup Order

```text
1. Base computer:
   python3 tools/basertkgps_5dk_survey.py

2. Rover Raspberry Pi Terminal 1:
   gps_rtk_node

3. Rover Raspberry Pi Terminal 2:
   arduino_bridge_node

4. Rover Raspberry Pi Terminal 3:
   Check /gps/rtk_status and /compass/heading_deg

5. Rover Raspberry Pi Terminal 4:
   Start waypoint_nav_node

6. Rover Raspberry Pi Terminal 5:
   Monitor /navigation/status and /navigation/distance_to_target_m
```
