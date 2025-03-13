---
layout: default
title: Checkpoint 1
nav_order: 2
parent: Checkpoints
grand_parent: Botlab
last_modified_at: 2025-03-13 17:37:48 -0500
---

**Edit 3/13/25:** Correct odometry filename, add information about where to modify for odometry.

To control the robot’s position, we need to tell it how to move. The basic command for moving the robot is the motor PWM, which determines how much effort the motors use.

However, it’s more practical to control the robot by setting its velocity directly. PWM values like give the motor "50% duty cycle" are less intuitive than setting mbot moving by "1 m/s" directly.  To do this, we control the speed of each wheel. There are two main ways:
- Open Loop: Set the motor PWM to achieve the desired speed without feedback. This is the least accurate.
- Feedback Control: Use encoder data to adjust the wheel speed with a feedback controller, making it more accurate.

In this lab, you’ll develop different robot controllers and collect data to compare their performance.

### Contents
* TOC
{:toc}

## Wheel Speed Calibration
The calibration program for the pico measures the robot’s motion to determine the polarity of the encoders and motors and then performs a wheel speed calibration. The data from the wheel speed calibration are printed to the terminal and stored in non-volatile memory on the MBot.

### Task 1.1
You should perform the calibration several times and record your results in a file for comparison. We will collect this data to determine the consistency of the MBot’s drive system across all bots.

Open one terminal with Minicom to view the calibration output, and use another terminal to flash the calibration firmware to the pico.

The calibration data consists of:
- Encoder Polarity: Describes whether the encoder's count increases or decreases when a positive PWM signal is applied to the motor.
- Motor Polarity: Refers to the relationship between the motor’s actual rotation direction and the commands it receives.
- Slopes and Intercepts: Define the linear relationship between the PWM duty cycles and the actual speeds of the wheels.

$$\text{PWM}=m \times \text{Speed} + b$$

{: .required_for_report }
Report the motor calibration with variance for the robot on a concrete floor (like in the lab)
<br><br> Questions to Consider:
<br> 1) How much variation is there in the calibration?
<br> 2) What do you think is the source of variation?


## Odometry
The odometry functions are implemented in `mbot_odometry.c` and `mbot_classic.c` and the estimated position and orientation of the MBot is calculated using dead reckoning equations with the mbot velocity based on wheel encoders only.

### Task 1.2
You need to test the provided odometry implementation by moving the robot known distances and turning by known angles. Verify if the odometry calculations match these values.

You can use `mbot_firmware/python/mbot_move_simple.py` to drive the MBot: drive 1m straight and see if x is approximately 1, or turn half a cycle to check if odometry theta is about pi. To verify odometry, run: `mbot lcm-spy --channels MBOT_ODOMETRY`, refer to the [MBot CLI Tools](/docs/botlab/how-to-guide/mbot-cli-tools) guide for more details.

{: .important }
Due to imprecise timings, your bot will *never* go the exact distance you tell it to in the mbot_move_simple script. The proper way to determine if your odometry is correct is to mark the starting position, measure how far it moved, then compare to your robot's internal odometry readings.

If you are unsatisfied with the accuracy of the provided odometry, you can include some of the features discussed in lecture, for example, gyrodometry, to improve the accuracy.

To apply the changes, you need to compile the mbot_firmware, and flash the .uf2 files to the control board same as we did in the system setup guide.
```bash
$ cd build
$ make
$ sudo mbot-upload-firmware flash mbot_classic_v1.1.0_enc48.uf2
```

{: .required_for_report }
Evaluate the performance of your odometry system with, and/or without, the improvements you made.

## Wheel Speed PID Controller
The file `mbot_firmware/src/mbot_classic.c` includes the main control function `mbot_loop(repeating_timer_t *rt)`. This function reads sensor data, estimates the current state, and updates motor commands. The motor controller operates in various modes, including PWM mode, motor velocity mode, and body velocity mode. For this task, you will edit the code inside the "MODE_MBOT_VEL" if statement, which corresponds to body velocity mode.

The current controllers utilize calibration data for driving the MBot at set speeds.

### Task 1.3
Your task is to develop enhanced wheel speed and body velocity controllers for more precise and effective robot movement.

The parameters for the controller are defined in a struct of type `mbot_ctlr_cfg_t`. To use the controller, implement the remaining functions in `mbot_controller.c` and call them.

Features you should consider adding/changing for your controller:
- Integrate Feed-Forward (FF) and Feedback (FB) controllers by summing their outputs. This approach allows the PID to focus solely on correcting the error between measured speed and the calibration function, potentially reducing or even eliminating the need for an integral term.
- Implement a low-pass filter on the wheel velocity estimates to minimize discretization noise, particularly at low speeds.
- Introduce acceleration and deceleration limits for the robot to prevent abrupt movements by filtering the command setpoints.

To test your updated controller, you can modify the Python script `mbot_firmware/python/mbot_move_simple.py`, using them to create commands that challenge and evaluate your controller's performance.

{: .required_for_report }
Provide a detailed description of your final controller, including a table of parameters (gains, filter parameters, etc.). Additionally, include an evaluation of the controller's performance.
<br><br>Questions to Consider:
<br> 1) How did you tune the parameters in your controller? What metric did you use to decide it was sufficiently tuned?
<br> 2) What additional features did you implement, and what were the effects?


## Motion Controller
The motion controller on the MBot is the interface between the planner and the low level controller. It reads in a planned path (`robot_path_t`) on the `CONTROLLER_PATH` channel and executes the planned motion of the robot. There is a working motion controller already implemented for you, so you do not need to write any code for this part. However, for better performance it may be wise to tweak some gains, and for maximum performance you may wish to write a more advanced controller.

{: .important }
The motion controller isn't running by default. If you ever want to send waypoints to the bot, such as in `drive_square.cpp`, you need to first run the program with `mbot_autonomy/build/mbot_motion_controller`. The program must be running in an active terminal window every time you send a path.

### Task 1.4
Study the motion controller in `mbot_autonomy/src/mbot/diff_motion_controller.cpp`. This program will run on the Raspberry Pi. The stock controller implements basic PID waypoint following, which is sufficient to follow a given path, although it may be wise to implement a more advanced motion controller such as carrot following or smooth pursuit.

{: .required_for_report }
Describe and document your motion control algorithm for getting between waypoints.
<br> 1) Include a plot of your robot’s dead reckoning estimated pose as the robot is commanded to drive a 1m square 4 times.
<br> 2) Include a plot of the robot's linear and rotational velocity as it drives one loop around the square

## Checkpoint Submission (Due 3/25/25)
<br>
<a class="image-link" href="/assets/images/botlab/checkpoints/checkpoint1-maze.png">
<img src="/assets/images/botlab/checkpoints/checkpoint1-maze.png" alt=" " style="max-width:600px;"/>
</a>

- Demonstrate your motion controller by having it drive the illustrated path in one of the mazes set up in the lab. Record a video of the robot attempting the path at a slow speed (~0.2m/s & pi/4 rad/s) and a high speed (~0.8m/s, pi rad/s) and provide a link to it.
    - The robot motion does NOT need to be perfect
- Produce a plot of the (x,y) odometry position as the robot executes the path at both speeds.
    - To record a log, use `lcm-logger -c MBOT_ODOMETRY my_lcm_log.log`
    - To parse a log and make a plot, refer to the first Botlab lecture slides.
- Write a short description of your controllers (1/2 page) and any features we should be aware of. It is OK to use the stock controllers.
