---
layout: default
title: Checkpoint 1
nav_order: 2
parent: Checkpoints
grand_parent: Botlab
last_modified_at: 2024-02-22 17:37:48 -0500
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


{: .required_for_report }
Report the motor calibration with variance for the robot on a concrete floor (like in the lab) 
<br><br> Questions to Consider:
<br> 1) How much variation is there in the calibration?
<br> 2) What do you think is the source of variation?


## Odometry
The odometry functions are implemented in `odometry.c` and the estimated position and orientation of the MBot is calculated using dead reckoning equations with wheel encoders only. 

### Task 1.2
You should test the odometry implementation we provided, by moving the robot by known distances and turning by known angles. If you are unsatisfied with the accuracy of the odometry, you can include some of the features discussed in lecture and additional IMU data to improve the accuracy.

{: .required_for_report }
Evaluate the performance of your odometry system with, and/or without, the improvements you made.

## Wheel Speed PID Controller
The file `mbot_firmware/src/mbot.c` includes the main control function `mbot_loop(repeating_timer_t *rt)`. This function reads sensor data, estimates the current state, and updates motor commands. The motor controller operates in various modes, including PWM mode, motor velocity mode, and body velocity mode.

Current controllers utilize calibration data for driving the MBot at set speeds. 

### Task 1.3
Your task is to develop enhanced wheel speed and body velocity controllers for more precise and effective robot movement.

Features you should consider adding/changing for your controller:
- Integrate Feed-Forward (FF) and Feedback (FB) controllers by summing their outputs. This approach allows the PID to focus solely on correcting the error between measured speed and the calibration function, potentially reducing or even eliminating the need for an integral term. 
- Implement a low-pass filter on the wheel velocity estimates to minimize discretization noise, particularly at low speeds.
- Introduce acceleration and deceleration limits for the robot to prevent abrupt movements by filtering the command setpoints.

To test your updated controller, you can modify the Python scripts: `drive_test.py`, `drive_square.py`, and `mbot_teleop.py`, using them to create commands that challenge and evaluate your controller's performance.

{: .required_for_report } 
Provide a detailed description of your final controller, including a table of parameters (gains, filter parameters, etc.). Additionally, include an evaluation of the controller's performance. 
<br><br>Questions to Consider:
<br> 1) How did you tune the parameters in your controller? What metric did you use to decide it was sufficiently tuned?
<br> 2) What additional features did you implement, and what were the effects?


## Motion Controller
The motion controller on the MBot is the interface between the planner and the low level controller. It reads in a planned path (`robot_path_t`) on the `CONTROLLER_PATH` channel and executes the planned motion of the robot. 

### Task 1.4
Implement this motion controller in `mbot/mbot_autonomy/src/mbot/motion_controller.cpp`. This program will run on the embedded computer.  

{: .required_for_report }
Describe and document your motion control algorithm for getting between waypoints. 
<br> 1) Include a plot of your robot’s dead reckoning estimated pose as the robot is commanded to drive a 1m square 4 times.
<br> 2) Include a plot of the robot's linear and rotational velocity as it drives one loop around the square

## Checkpoint Submission
<br>
<a class="image-link" href="/assets/images/botlab/checkpoints/checkpoint1-maze.png">
<img src="/assets/images/botlab/checkpoints/checkpoint1-maze.png" alt=" " style="max-width:600px;"/>
</a>

- On the Google Form:
    - Report the status of your teams robots
    - Submit your motor calibrations and statistics for any robots built
- Demonstrate your motion controller by having it drive the illustrated path in one of the mazes set up in the lab. Record a video of the robot attempting the path at a slow speed (~0.2m/s & pi/4 rad/s) and a high speed (~0.8m/s, pi rad/s) and provide a link to it.
    - The robot motion does NOT need to be perfect
- Produce a plot of the (x,y) odometry position as the robot executes the path at both speeds.
- Write a short description of your controllers (1/2 page) and any features we should be aware of. It is OK to use the stock controllers.
