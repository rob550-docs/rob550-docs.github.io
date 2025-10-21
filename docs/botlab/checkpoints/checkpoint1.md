---
layout: default
title: Checkpoint 1
nav_order: 2
parent: Checkpoints
grand_parent: Botlab
last_modified_at: 2025-10-21 11:37:48 -0500
---


{: .note}
We don’t need the LiDAR for Checkpoint 1. If you want to stop it from spinning and draining the battery, you can unplug the three jumper wires from the control board. When plugging it back in, be very careful, double-check the cable connections to avoid shorting the LiDAR.

### Contents
* TOC
{:toc}

## Preparation

In the system setup, you’re provided with two precompiled `.uf2` files. Those are only for testing whether all the components are functioning correctly and are **not related to any of the checkpoints.**

First pull from the uf2 repo, get the latest checkpoint 1 uf2 file.
```bash
cd ~/botlab_uf2
git pull
```
- You should now have "mbot_classic_ros_checkpoint1.uf2".

Then fork the firmware repository and clone it to your MBot. **You don’t need to modify the firmware codebase for checkpoint 1**, you’ll only use some of the configuration files from this repository.
1. Fork the [mbot_firmware_ros repository](https://gitlab.eecs.umich.edu/rob550-f25/mbot_firmware_ros) to your GitLab group.
2. Clone mbot_firmware_ros to your mbot's home directory:
   ```bash
    cd ~
    git clone your_url
    ```

## Task 1.1 Wheel Calibration
The calibration program determines the polarity of the encoders and motors, then performs a wheel speed calibration. The resulting data are printed to the terminal and stored in the MBot’s non-volatile memory.

The calibration data consists of:
- Encoder Polarity: Describes whether the encoder's count increases or decreases when a positive PWM signal is applied to the motor.
- Motor Polarity: Refers to the relationship between the motor’s actual rotation direction and the commands it receives.
- Slopes and Intercepts: Define the linear relationship between the PWM duty cycles and the actual speeds of the wheels.

$$\text{PWM}=m \times \text{Speed} + b$$

### TODO
Open one terminal with Minicom to monitor the calibration output, and use another terminal to flash the calibration firmware to the Pico. Run the calibration several times and collect the resulting data for analysis. This task focuses on analyzing the system’s performance.

```bash
# terminal 1
sudo minicom -D /dev/ttyACM0 -b 115200
# terminal 2
cd ~/botlab_uf2
sudo mbot-upload-firmware flash mbot_calibrate_classic_v1.1.1.uf2
```

Once you’ve finished collecting data, remember to flash **the Checkpoint 1 firmware** to the control board.
```bash
cd ~/botlab_uf2
sudo mbot-upload-firmware flash mbot_classic_ros_checkpoint1.uf2
```

{: .required_for_report }
Report the motor calibration with variance for the robot on a concrete floor (like in the lab).
<br><br> Questions to Consider:
<br> 1) How much variation is there in the calibration?
<br> 2) What do you think is the source of variation?


## Task 1.2 PID Tuning
There are 3 drive modes and 3 control modes in firmware.

- **PWM Control**
- **Wheel Velocity Control**
  - Feedforward (FF) Control 
  - PID Control
  - Feedforward + PID Control 
- **Robot Velocity Control**
  - Feedforward (FF) Control
  - PID Control
  - Feedforward + PID Control 

The **drive mode is automatically selected** based on the type of control command you send:
- If you publish to `/cmd_vel`, **Robot Velocity Control** mode will be selected, meaning the robot will interpret commands as body velocities.

The **control mode is automatically selected** when you run calibration script, the default is "Feedforward + PID Control ". The calibration will read this header file (`mbot_firmware_ros/include/config/mbot_classic_default_pid.h`) and write the default values to FRAM.

You can tune the PID gains via the ROS parameter server at runtime.

### TODO
Tune the PID values to achieve a desired system response.

**How to tune?**
1. First, modify the values in `include/config/mbot_classic_pid.yaml`. There are pre-tuned values in there, the same as the `mbot_classic_default_pid.h.`
2. After modifying the values in the yaml file, run the command below to load the new configs:
    ```bash
    cd ~/mbot_firmware_ros
    ros2 param load /mbot_control_node include/config/mbot_classic_pid.yaml
    ```
3. If you want to check whether the values are actually loaded, run:
    ```bash
    ros2 param dump /mbot_control_node 
    ```
    - If you see the values you set, it means the new PID gains have been applied to the MBot and written to the FRAM. 
    - The values will persist after rebooting. However, if you run the calibration again, it will overwrite your new PID gains, and you’ll need to reload the YAML file to restore them.

**How to test?**
- We provide a simple python script: `mbot_firmware_ros/python-tests/test_wheel_pid.py`. It will drive the robot and print the target vs. real speed to the terminal. Use this file as a starting point, modify it to make comparisons, and collect data for plots.

{: .required_for_report }
Plots of time vs. velocity with robot driving in FF model vs. PID controller model vs. FF + PID controller
<br><br> Questions to Consider:
<br> 1) Which wheel controller performs the best and the worst, why?
<br> 2) Is there any improvement we can make?

---

Below this point is still under editing
{: .fs-5 .text-red-200 .fw-500}


## Task 1.3 Motion Controller

### TODO

**How to test?**

{: .required_for_report }
Describe and document your motion control algorithm for getting between waypoints.
<br><br>Questions to Consider:
<br> 1) Include a plot of your robot’s estimated pose as the robot is commanded to drive a 1m square 4 times.
<br> 2) Include a plot of the robot’s linear and rotational velocity as it drives one loop around the square


## Checkpoint Submission
<br>
<a class="image-link" href="/assets/images/botlab/checkpoints/checkpoint1-maze.png">
<img src="/assets/images/botlab/checkpoints/checkpoint1-maze.png" alt=" " style="max-width:600px;"/>
</a>

- Demonstrate your motion controller by having it drive the illustrated path in one of the mazes set up in the lab. Record a video of the robot attempting the path at a slow speed (~0.2m/s & pi/4 rad/s) and a high speed (~0.8m/s, pi rad/s) and provide a link to it.
    - The robot motion does NOT need to be perfect
- Produce a plot of the (x,y) odometry position as the robot executes the path at both speeds.
- Write a short description of your controllers (1/2 page) and any features we should be aware of. It is OK to use the stock controllers.
