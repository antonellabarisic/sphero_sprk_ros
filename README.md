# sphero_sprk_ros

This repository contains a Python driver for control and communication with Sphero SPRK+ within ROS. The purpose of this driver is to provide a platform for further research and development of custom applications for Sphero SPRK+ robot.

Example of use with Reynolds flocking algorithm can be found at https://github.com/mkrizmancic/sphero_formation

Note: The driver can be used without ROS.
## Table of contents
- [Installation](#Installation)
  - [Requirements](#Requirements)
  - [Download and build](#download-build)
- [Usage](#Usage)
- [Package description](#pckg)
  - [Subscribed topics](#sub)
  - [Published topics](#pub)
  - [Scope of functionalities](#Scope)


## <a name="Installation"></a> Installation
```sphero_sprk_ros``` was tested on Ubuntu 16.04/ROS Kinetic and Ubuntu 18.04/ROS Melodic running Python 2.7. Ubuntu 20.04/ROS Noetic running Python 3.8 is partially supported. It is possible to send commands to the robot (roll, LED color, stabilization, manual calibration, back LED), but data streaming (IMU and locator) doesn't currently work.

Due to BLE connection problems connected with kernel versions, we verified following versions of the kernel (the list is updated as new kernel versions are verified):
- 4.4.0-21
- 4.4.0-164
- 5.0.0-23
- 5.0.0-29 (Recommended)
- 5.4.0-48 (Recommended)

### <a name="Requirements"></a> Requirements
- Ubuntu 16.04, Ubuntu 18.04 or Ubuntu 20.04
- ROS Kinetic, ROS Melodic or ROS Noetic
- Python 2.7 (full support) or Python 3.8+ (only sending commands, no data streaming)
- bluepy (bluez)
  ```shell script
  $ sudo apt install bluez
  $ pip install bluepy
  ```

### <a name="download-build"></a> Download and build
```
$ cd <path_to_your_catkin_ws>/src
$ git clone https://github.com/antonellabarisic/sphero_sprk_ros.git 
$ cd sphero_sprk_ros 
$ git checkout noetic-devel 
$ cd <path_to_your_catkin_ws>
$ catkin_make
$ source <path_to_your_catkin_ws>/devel/setup.bash
```
**Important note:** <br>
```catkin build``` is the preferred tool for building the package, but it seems it doesn't work out of the box on Ubuntu 20.04, so you can use ```catkin_make```. However, these tools are not interchangeable. Once the workspace is build using one of them, the other can't be used in that workspace.

## <a name="Usage"></a> Usage

Simple launch file for one Sphero SPRK+ is provided. You need to specify MAC address of Sphero and a custom name. Parameter ```data_stream``` (possible values are "All" or "Locator") should be left on "None" as data streaming is not supported in Python 3.x.
```
roslaunch sphero_sprk_ros one_sphero.launch
```

You can easily add multiple robots to launch file or use:
```
roslaunch sphero_sprk_ros drivers.launch
```
In the second launch file, you specify the number of Sphero robots. The first _n_ addresses from [cfg/sphero_addresses.txt](cfg/sphero_addresses.txt) will be used to launch _n_ nodes. If _n_=1, both launch files are equivalent.

## <a name="pckg"></a> Package description

### <a name="sub"></a> Subscribed topics
- ```/sphero_0/cmd_vel```
  - Type: geometry_msgs/Twist
- ```/sphero_0/set_color```
  - Type: std_msgs/ColorRGBA
- ```/sphero_0/set_heading```
  - Type: std_msgs/Float32
- ```/sphero_0/set_angular_velocity```
  - Type: std_msgs/Float32
- ```/sphero_0/disable_stabilization```
  - Type: std_msgs/Bool
- ```/sphero_0/manual_calibration```
  - Type: std_msgs/Bool

### <a name="pub"></a> Published topics
- ```/sphero_0/imu```
  - Type: sensor_msgs/Imu
- ```/sphero_0/odom```
  - Type: nav_msgs/Odometry
- ```/sphero_0/diagnostics```
  - Type: diagnostic_msgs/DiagnosticArray

### <a name="Scope"></a> Scope of functionalities
- ```connect```, ```ping```, ```sleep```, ```disconnect```
- ```roll```
- ```set_power_notify```
- ```get_power_state```
- ```get_device_name```
- ```set_rgb_led```
- ```set_back_led```
- ```set_stabilization```
- ```set_heading```
- ```set_filtered_data_strm```
- ```set_locator```
- ```read_locator```
