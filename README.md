# sphero_sprk_ros

This repository contains a Pyhton driver for control and communication with Sphero SPRK+ within ROS. The purpose of this driver is to provide a platform for further research and development of custom applications for Sphero SPRK+ robot.

Example of use with Reynolds flocking algorithm can be found at https://github.com/mkrizmancic/sphero_formation

Note: The driver can be used without ROS.
## Table of contents
- [Installation](#Installation)
  - [Requirements](#Requirements)
- [Usage](#Usage)
- [Package description](#pckg)
  - [Subscribed topics](#sub)
  - [Published topics](#pub)
  - [Scope of functionalities](#Scope)


## <a name="Installation"></a> Installation
```sphero_sprk_ros``` is tested on Ubuntu 16.04/ROS Kinetic and Ubuntu 18.04/ROS Melodic. Due to BLE connection problems connected with kernel versions, we verified following versions of the kernel (the list is updated as new kernel versions are verified):
- 4.4.0-21
- 4.4.0-164
- 5.0.0-23
- 5.0.0-29 (Recommended)
- 5.4.0-48 (Recommended)

### <a name="Requirements"></a> Requirements
- Ubuntu 16.04 / Ubuntu 18.04
- ROS Kinetic / ROS Melodic
- Python 2.7
- bluepy (bluez)
  ```shell script
  $ sudo apt install bluez
  $ pip install bluepy
  ```

Install ```sphero_sprk_ros```:
```
$ cd <path_to_your_catkin_ws>/src
$ git clone
$ cd catkin_ws
$ catkin build
$ source <path_to_your_catkin_ws>/devel/setup.bash
```
## <a name="Usage"></a> Usage

Simple launch file for one Sphero SPRK+ is provided. You need to specify MAC address of Sphero, custom name and parameter ```data_stream``` ("All" or "Locator").

```
roslaunch sphero_sprk_ros one_sphero.launch
```
You can easily add multiple robots to launch file.

## <a name="pckg"></a> Package description

### <a name="sub"></a> Subscribed topics
- ```/sphero_1/cmd_vel```
  - Type: geometry_msgs/Twist
- ```/sphero_1/set_color```
  - Type: std_msgs/ColorRGBA
- ```/sphero_1/set_heading```
  - Type: std_msgs/Float32
- ```/sphero_1/set_angular_velocity```
  - Type: std_msgs/Float32
- ```/sphero_1/disable_stabilization```
  - Type: std_msgs/Bool
- ```/sphero_1/manual_calibration```
  - Type: std_msgs/Bool

### <a name="pub"></a> Published topics
- ```/sphero_1/imu```
  - Type: sensor_msgs/Imu
- ```/sphero_1/odom```
  - Type: nav_msgs/Odometry
- ```/sphero_1/diagnostics```
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



