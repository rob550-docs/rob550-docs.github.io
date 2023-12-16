---
layout: default
title: Checkpoints
nav_order: 2
parent: Armlab
has_children: true
last_modified_at: 2023-12-12 14:37:48 -0500
---

The overall goal of these lab tasks is to develop computer vision algorithms and work with the kinematics of a 5-DOF manipulator to detect and manipulate blocks on a board within the reach of the robot arm. 

### Overview
Armlab will have two fairly separate components that you can work on in parallel: 
- Computer vision, which involves
    - Camera calibration for an RGB-D camera
    - Block detection
    - Color detection
- Kinematics, which involves
    - Forward Kinematics (FK) using DH tables or PoX
    - Inverse Kinematics (IK)

We encourage you to try and distribute tasks to progress faster through the project but remember that you will be quizzed on all the material after the end of the lab. Keep each other updated and informed on how different pieces worked. 
We will now cover the specific material that you will be working with for the block detection, and the setup in which the 5-DOF arm and RGB-D camera are set up.

### Board

<a class="image-link" href="/assets/images/armlab/checkpoints/board.png">
<img src="/assets/images/armlab/checkpoints/board.png" alt="" style="max-width:600px;"/>
</a>

The 5-DOF arm is placed on a board that has a 50 mm grid. The robot is placed at a specific location on the grid with a defined origin to make it easier to define a coordinate frame. There are additionally 4 preset locations that have Apriltags, which will be used for the camera calibration task.

The origin of the robot frame is shown in Figure, with +z being out of the board. When locations are referenced in the lab document, they will be in reference to the robot frame shown in this figure. 

Note that the horizontal grid lines do not line up with the origin of the robot frame, but are offset by 25mm. Apriltags are placed on the board at the indicated locations 1-4, with Tag IDs printed on the sticker.  
