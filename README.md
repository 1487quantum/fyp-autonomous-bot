# jaguar-bot
This repository contains the codes & drivers used for the autonomous mobile robot project (FYP) to perform autonomous navigation. The *conifer-dashboard* (web dashboard) is used to display the telemetry and relevant information of the robot via a local web server with *NodeJS*. (The repo is located over here: https://github.com/1487quantum/conifer-dashboard)

## System
- Ubuntu 14.04 LTS (64-bit)
- ROS Indigo

## ROS Packages
### System
The following ROS packages are required to be installed system wide:
- Gmapping: ros-indigo-gmapping
- Map server: ros-indigo-map_server
- AMCL: ros-indigo-amcl
- Move base: ros-indigo-move-base
- Camera Info Manager (Py): ros-indigo-camera-info-manage-py

To do so, simply run the following command, replacing *package_name* with the ones you need.
```
$ sudo apt-get install ros-indigo-[package_name]
```

### Local
All the packages in this repo should be cloned & place into your workspace.
```
$ git clone https://github.com/1487quantum/jaguar-bot.git
```

## Directories
An overview of the purpose of the various directories:
- *joy*: Joystick driver (Logitech)
- *teleop_twist_joy*: Process /joy -> /cmd_vel
- *diff_drive_controller*: Process /cmd_vel -> /joint_trajectory
- *kangaroo_x2_driver*: Process /joint_trajectory -> /joint_state, controls the motor
- *lms1xx_driver*: Lidar Driver (LMS111)
- *robot_core*: Main control launch/config files to run the robot is here
- *axis_camera*: Axis Web camera

## Troubleshooting
- ** The joystick is detected by the computer (checked via *lsusb*) , but why is it not recognised as a port in */dev/input* as *js0*? **

  - Most probably the linux kernel was not loaded. You can load the kernel by using the *modprobe* command [1]:
```
$ sudo modprobe xpad
```
After that, you should be able to find *js0* in */dev/input/*

- **[kangaroo_driver_node] process has _died_**     
  - If the following error is seen:
  ```
[ERROR] [1496303682.359675330]: Failed to get the Data.
*** stack smashing detected ***: ~/catkin_ws/devel/lib/kangaroo_driver/kangaroo_driver_node terminated
```
It might be due to a wrongly specified USB port. Try changing the port number in the launch file _param_ for the _kangaroo_ node to either _/dev/ttyUSB0_, _/dev/ttyUSB1_, etc.
```
<param name="port" value="/dev/ttyUSB1" />
```
If the IMU is used & the following error occurs, try swapping the USB ports of the IMU and the kangaroo node instead.



## References
- [1]: http://nomoreterminals.blogspot.sg/2013/12/how-to-set-up-logitech-f310-on-ubuntu.html
