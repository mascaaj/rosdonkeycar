# ROS1 stack for the Donkey Car platform (rosdonkey)

This repository holds the code for the hardware and simulation bring up of the donkeycar platform with ROS1.
It is in development and has been tested with Kinetic. It will move to Noetic by EOY and ROS2 by 2023 (hopefully)
The intent was to provide an ackerman steer platform for software development along with hobby grade sensors.

## Description :

### Vehicle platform
 - Desert Rover from Traxxis ($140 from amazon)

### Sensors:
 - Pi camera v2 (1280x960)
 - BNO055 imu (9 dof absolute orientation)
 - KY-003 Hall Effect Magnetic Sensor (Onyehn)
 - Tfmini Plus Laser Range finder (Benwake)
 - MG995 servo (Laser -> LaserScan)

### Interfaces and Compute
 - i2cpwm board 16 Channel (Sunfounder)
 - Raspberry Pi 4B (8GB, bought before it became $$$)
 - Logitech f310 wireless controller

### Operating System
 - [Ubiquity Robotics - Ubuntu 16.04 + ROS Kinetic](https://downloads.ubiquityrobotics.com/pi.html)