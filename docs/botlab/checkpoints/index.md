---
layout: default
title: Checkpoints
nav_order: 3
parent: Botlab
has_children: true
last_modified_at: 2025-10-17 18:37:48 -0500
---

In this lab, you will be working with the ROS2 MBot Classic. Each MBot moves with differential drive, 2 parallel wheels with a rear caster.  Each motor is equipped with a magnetic wheel encoder. 

The robot also has a scanning 2D Lidar, and a MEMS 3-axis IMU. The MBot has a Raspberry Pi 5. The low level motor control is implemented on the MBot Control Board on a Raspberry Pi RP2040 based microcontroller.


### Overview
#### ACTION
- Cascade Control & PID
- 3-DOF rigid-body coordinate transforms
- Planar kinematics of a differential-drive ground robot
- Motion models with uncertainty

#### PERCEPTION 
- Quadrature Encoders
- MEMS Inertial Measurement Unit
- 2D LIDAR Rangefinders
- Camera and Fiducial Marker Detection

#### REASONING
- Monte Carlo Localization 
- Simultaneous Localization and Mapping
- A* search
- Path planning


### Structure

Part 1: 
- Assemble the MBot

Part 2:
- Tune the feedback controller to control the motor speed
- Design a controller to move the robot based on velocity commands

Part 3:
- Implement 2D mapping using the 2D Lidar
- Build a SLAM system and map and localize in an environment
- Build a path planner to navigate in the map

Part 4:
- Use OpenCV and apriltags to find obstacles and targets in the environment
- Implement a Potential Field Planner and Visual Servoing


This is a team assignment. Your team must complete it together, collectively participating on each component.
