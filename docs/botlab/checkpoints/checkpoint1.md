---
layout: default
title: Checkpoint 1
nav_order: 2
parent: Checkpoints
grand_parent: Botlab
last_modified_at: 2025-10-22 21:16:48 -0500
---


{: .note}
We don’t need the LiDAR for Checkpoint 1. If you want to stop it from spinning and draining the battery, you can unplug the three jumper wires from the control board. When plugging it back in, be very careful, double-check the cable connections to avoid shorting the LiDAR.

### Contents
* TOC
{:toc}

## Preparation

In the system setup, you’re provided with two precompiled `.uf2` files. They are for testing whether all components are functioning correctly. We will continue releasing additional `.uf2` files for each checkpoint, so please pay attention to the filenames.

First pull from the uf2 repo, get the latest checkpoint 1 uf2 file.
```bash
cd ~/botlab_uf2
git pull
```
- You should now have "mbot_classic_ros_checkpoint1.uf2".

Then fork the firmware repository and clone it to your MBot for now.
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
- Motor Polarity: Refers to the relationship between the motor’s actual rotation direction and the command it receives.
  - For example: if the right wheel’s polarity is **-1**, and you command it to “rotate forward at 1 m/s,” the wheel will rotate forward, which might seem counterintuitive given the negative polarity.
    - The reason is that the z-axes of the wheels point outward. According to the right-hand rule, the positive rotation direction for the right wheel corresponds to a backward rotation relative to the MBot’s body frame.
- Slopes and Intercepts: Define the linear relationship between the PWM duty cycles and the actual speeds of the wheels.

$$\text{PWM}=m \times \text{Speed} + b$$

### TODO
Open one terminal with Minicom to monitor the calibration output, and use another terminal to flash the calibration firmware to the Pico. Run the calibration several times and collect the resulting data for analysis. Also record your **encoder polarities**, you will need it for Task 1.3. This task focuses on analyzing the system’s performance.

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
1. First, modify the values in `include/config/mbot_classic_pid.yaml`. 

    This file contains pre-tuned values that are the same as those in `mbot_classic_default_pid.h`. However, these default parameters may not be compatible with your specific robot and could cause your MBot to perform poorly when using the teleop controller.
2. After modifying the values in the yaml file, run the command below to load the new configs:
    ```bash
    cd ~/mbot_firmware_ros
    ros2 param load /mbot_control_node include/config/mbot_classic_pid.yaml
    ```
3. If you want to check whether the values are actually loaded, run:
    ```bash
    ros2 param dump /mbot_control_node 
    ```
    - If you see the values you set, it means the new PID gains have been applied to the MBot and written to the FRAM. You can run test now!
    - The PID values will persist after rebooting. However, if you run the calibration again, the calibration script will overwrite your tuned PID gains, and you’ll need to reload the YAML file to restore them.
      - After you are satisfied with your PID gains, if you really want to check whether the gains persist after rebooting, you can verify this as follows:
        - Open a terminal and start Minicom.
        - In another terminal, flash the firmware. Before the data table is printed, the firmware will first display all the saved parameters, check the PID values there.

**How to test?**
- We provide a simple python script: `mbot_firmware_ros/python-tests/test_wheel_pid.py`. It will drive the robot and print the target vs. real speed to the terminal. Use this file as a starting point, modify it to make comparisons, and collect data for plots.

{: .required_for_report }
Plot of time vs. velocity with robot responding to a step command of 0.5 m/s for the FF model, PID controller model and FF + PID controller (3 traces on one plot).
<br><br> Questions to Consider:
<br> 1) Which wheel controller performs the best and the worst, why?
<br> 2) Is there any improvement we can make?


## Task 1.3 Odometry

Odometry estimates your robot's position and orientation by integrating wheel encoder measurements over time. You'll subscribe to encoder messages and use differential drive kinematics to calculate how the robot moves.

{: .note}
Please rememeber, you should flash the `mbot_classic_ros_checkpoint1.uf2` to you pico, not `mbot_classic_ros_v1.1.1.uf2`.

### TODO
1. First fork the [mbot_ros_labs](https://gitlab.eecs.umich.edu/rob550-f25/mbot_labs_ws) repository to your group. This repository contains the code for checkpoints.
2. Create a workspace directory named `mbot_ros_labs`, and clone the forked repository into a subdirectory called `src`:
  ```bash
  cd ~
  mkdir mbot_ros_labs
  cd ~/mbot_ros_labs
  git clone your_url src
  ```
  - Please follow the exact name used in this document. Some paths (for logging and other files) are hardcoded in the codebase, so directory names must match exactly.
3. Implement the wheel velocity calculation, robot body velocity calculation, and odometry calculation in the file `mbot_setpoint/src/odom.cpp`. You can search for the keyword “TODO”, all the functions you need to complete are marked with “TODO” and numbered in order. Follow the numbering sequence as you implement them.
4. Once you’ve finished writing your code, compile it:
  ```bash
  cd ~/mbot_ros_labs
  colcon build
  source install/setup.bash
  ```
  - Always remember to source your workspace after building!
  - Or you can add the sourcing command to your bash configuration:
    ```bash
    cd ~/mbot_ros_labs
    echo "source $PWD/install/local_setup.bash" >> ~/.bashrc
    ```

**How to test?**
1. Place robot at a marked starting point and run the odom node.
  ```bash
  ros2 run mbot_setpoint odom
  ```
2. Use teleop to drive the robot to a known length and angle.
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```
3. Then check your odometry values
  ```bash
  ros2 topic echo /odom --field pose
  ```
4. If you want to achieve more accurate odometry, consider adding enhancement code. For example, you could incorporate gyro readings and implement gyrodometry. This will require you to add an IMU subscriber on your own.

{: .required_for_report }
Evaluate the performance of your odometry system.

## Task 1.4 Motion Controller
The motion controller will take a series of waypoints and navigate through them. 

In this task, you will implement a rotate-translate-rotate controller. This is a high-level strategy that uses a PID controller to precisely execute each movement (rotating to the goal, driving forward, and rotating to the final orientation).

### TODO
1. Implement the controller in the file `mbot_setpoint/src/motion_controller_diff.cpp`.
You can search for the keyword “TODO”, all the functions you need to complete are marked with “TODO” and numbered. Follow the numbering sequence as you implement them.
2. Once you’ve finished writing your code, compile it:
  ```bash
  cd ~/mbot_ros_labs
  colcon build
  source install/setup.bash
  ```

**How to test?**
1. Place robot at a marked starting point and run the odom node.
  ```bash
  ros2 run mbot_setpoint odom
  ```
2. Run the motion controller
  ```bash
  ros2 run mbot_setpoint motion_controller_diff
  ```
3. Publish the waypoints. By default, this node publishes a square with 1-meter sides:
  ```bash
  ros2 run mbot_setpoint square_publisher
  ```
4. The motion controller node will also log some values and save them in the `logs` directory.
You can run the Python script to plot the data, the generated images will also be saved in the `logs` directory.
  ```bash
  python3 motion_controller_plot.py
  ```

{: .required_for_report }
Describe your motion control algorithm.
<br><br>Questions to Consider:
<br> 1) Include a plot of your robot’s estimated pose as the robot is commanded to drive a 1m square 4 times.
<br> 2) Include a plot of the robot’s linear and rotational velocity as it drives one loop around the square


## Checkpoint Submission
<br>
<a class="image-link" href="/assets/images/botlab/checkpoints/checkpoint1-maze.png">
<img src="/assets/images/botlab/checkpoints/checkpoint1-maze.png" alt=" " style="max-width:600px;"/>
</a>

- Run you motion controller to drive the path show in the image above. Include the following items in your submission:
  1. Record a video of the robot attempting the path at a slow speed (~0.2m/s & pi/4 rad/s) and a high speed (~0.8m/s, pi rad/s) and provide a link to it. The robot motion does NOT need to be perfect
  2. Include a plot of the (x, y) odometry position as the robot executes the path at both speeds.
  3. Write a short description of your controllers (1/2 page) and any features we should be aware of.