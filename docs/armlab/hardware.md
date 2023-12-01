---
layout: default
title: Hardware
nav_order: 4
parent: Armlab
last_modified_at: 2023-11-30 14:37:48 -0500
---

> This post will include details on the 5-DOF robotic arm, the motors on the arm, the camera used for detection, and the communication protocol of the hardware. 

### Contents
* TOC
{:toc}

### RX200 Arm 
<a class="image-link" href="https://www.trossenrobotics.com/Shared/Images/Product/ReactorX-200-Robot-Arm/RX200.jpg">
<img src="https://www.trossenrobotics.com/Shared/Images/Product/ReactorX-200-Robot-Arm/RX200.jpg" alt=" " style="max-width:300px;"/>
</a>

The ReactorX 200 Robot Arm is a 5 DOF manipulator with 7 Dynamixel servo motors. Further information about the manipulator is available at the manufacturer's website [here](http://support.interbotix.com/html/specifications/rx200.html).  

We’d like to point out that we have made a few modifications:
1. The original XL430 wrist motor is replaced with an XM430. 
2. We made modifications towards improving the gripper.


### Dynamixel Motors
Each joint on the RX200 arm consists of a DC motor to enable rotation around that joint. The two types of DC motors on the RX200 are listed below with their specification and links to their datasheets.

| Motor | Image | Info |
| ----- | ----- | ---- |
| XM430-W350-T | <a class="image-link" href="https://emanual.robotis.com/assets/images/dxl/x/x_series_product.png"><img src="https://emanual.robotis.com/assets/images/dxl/x/x_series_product.png" alt=" " style="max-width:150px;"/></a>| [Data Sheet](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/) |
| XL430-W250-T | <a class="image-link" href="https://emanual.robotis.com/assets/images/dxl/x/xl430_product.png"><img src="https://emanual.robotis.com/assets/images/dxl/x/xl430_product.png" alt=" " style="max-width:150px;"/></a>| [Data Sheet](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/) |


### Serial Interface & Power
Dynamixel servos use a three-wire connection and are all wired up in parallel.  
The two connectors on either side of the motor are identical, and the order in which the motors are connected does not matter.  

<a class="image-link" href="https://emanual.robotis.com/assets/images/dxl/x/x_series_ttl_pin.png">
<img src="https://emanual.robotis.com/assets/images/dxl/x/x_series_ttl_pin.png" alt=" " style="max-width:300px;"/>
</a>

- Pin 1 is the ground reference pin
- Pin 2 is the power
- Pin 3 is the data pin, carries information using a [half-duplex asynchronous serial protocol](https://learn.sparkfun.com/tutorials/serial-communication/all#:~:text=Half%2Dduplex%20communication%20means%20serial,back%20to%20the%20controlling%20device.), meaning transmit and receive data are carried on the same physical wire at a fixed (1Mbps) baud rate.


### Realsense Sensor
We will use the Realsense Sensor L515 for computer vision and perception tasks. L515 requires a USB 3.0 connection and consists of an RGB camera and a solid-state LIDAR. To interface with the sensor and capture the images needed for our project, we will be using the ROS (Robot Operating System) Realsense driver, a piece of software specifically designed for this task.
