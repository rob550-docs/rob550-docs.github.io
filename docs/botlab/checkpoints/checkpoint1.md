---
layout: default
title: Checkpoint 1
nav_order: 2
parent: Checkpoints
grand_parent: Botlab
last_modified_at: 2023-12-23 14:37:48 -0500
---

We want to be able to control the robot’s position, but to do this we need to be able to tell the robot how to move. The control signal we have to move the robot is a motor PWM command, which effectively commands how much effort we wish the motors to produce.  

However, it would be most convenient to be able to command the velocities of the robot from a higher level in order to move the robot in the environment. One simple way to control the robot speed is to control the speed of each wheel. This can be accomplished in a few different methods:
- One method is to set the motor pwm based on a function that you determine will produce the desired wheel speed. This is called an *open loop* because there is no feedback. This is the least accurate way of controlling the speed.  
- A better method would be to use encoder data to control the wheel speed using a feedback controller. The goal of this part of the lab will be to build an increasingly sophisticated robot controller, and at the end take quantitative data to compare them.

### Contents
* TOC
{:toc}

## Wheel Speed Calibration
The calibration program for the pico measures the robot’s motion to determine the polarity of the sensors and motors and then performs a wheel speed calibration. The data from the wheel speed calibration are printed to the terminal and stored in non-volatile memory on the MBot.

### Task 1.1
You should perform the calibration several times and record your results in a file for comparison. We will collect this data to determine the consistency of the MBot’s drive system across all bots.

Required For the Report:
- Report the motor calibration with variance for the robot on a concrete floor (like in the lab)

Questions to Consider:
- How much variation is there in the calibration?
- What do you think is the source of variation?


## Odometry
The odometry functions are implemented in `odometry.c` and the estimated position and orientation of the MBot is calculated using dead reckoning equations with wheel encoders only. 

### Task 1.2
You should test this implementation, by moving the robot by known distances and turning by known angles. If you are unsatisfied with the accuracy of the odometry, you can include some of the features discussed in lecture and additional IMU data to improve the accuracy.

Required For the Report:
- Evaluate the performance of your odometry system with, and/or without, the improvements you made.

## Wheel Speed PID Controller


## Motion Controller


## Checkpoint Submission
