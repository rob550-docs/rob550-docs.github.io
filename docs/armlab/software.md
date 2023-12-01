---
layout: default
title: Software
nav_order: 5
parent: Armlab
last_modified_at: 2023-11-30 14:37:48 -0500
---

> In this section, we will describe the overall software structure you will be working with. 

<a class="image-link" href="/assets/images/armlab/software/architecture.png">
<img src="/assets/images/armlab/software/architecture.png" alt="Overview of the software architecture" style="max-width:800px;"/>
</a>

### Contents
* TOC
{:toc}

### ROS2 
ROS 2 (Robot Operating System 2) is an open source software development kit for robotics applications. The purpose of ROS 2 is to offer a standard software platform to developers across industries. ROS 2 builds on the success of ROS 1, which is used today in myriad robotics applications around the world.

ROS2 provides a high level of abstraction by allowing you to treat hardware components (like a robotic arm, motors, or sensors) as if they're software modules or objects within your program. This allows for a high degree of modularity and reusability, making it easier to develop and test your software. 

In the Armlab we are using ROS2 Humble, more can be learned from the [official doc](https://docs.ros.org/en/humble/index.html).


### Realsense2_camera
This is the ROS driver node for the Realsense camera.  It enables us to stream the camera output as a ROS message. You can find information on the Intel RealSense [GitHub](https://github.com/IntelRealSense/realsense-ros).


### interbotix_sdk
This is the ROS driver node for the Interbotix Robot Arm. This node allows us to subscribe to topics relating to the joint states, representing the angle of the joints read from the motors. You can find more information on the [GitHub](https://github.com/Interbotix/interbotix_ros_manipulators).