---
layout: default
title: Checkpoint 1
nav_order: 2
parent: Checkpoints
grand_parent: Botlab
last_modified_at: 2025-10-23 12:16:48 -0500
---

Oct. 23 Update
{: .fs-5 .text-red-200 .fw-500}

- **If you just finished setup**: You are not affected, please proceed as it is.
- **If you are working on Task 1.1**: Keep going. Once you’re done, revisit the Preparation section to compile the latest .uf2 files, then proceed to Task 1.2.
- **If you are working on Task 1.2**: Revisit the Preparation section to compile the latest .uf2 files. Then review Task 1.2 again. You’ll notice that the command used to upload the .uf2 files has changed. This is because we’re no longer using the provided precompiled files; instead, you need to compile them yourself from the source code.

---

{: .note}
We don’t need the LiDAR for Checkpoint 1. If you want to stop it from spinning and draining the battery, you can unplug the three jumper wires from the control board. When plugging it back in, be very careful, double-check the cable connections to avoid shorting the LiDAR.

### Contents
* TOC
{:toc}

## Preparation
Fork the firmware repository to your group and clone it to your MBot.
1. Fork the [mbot_firmware_ros repository](https://gitlab.eecs.umich.edu/rob550-f25/mbot_firmware_ros) to your GitLab group.
2. Clone mbot_firmware_ros to your mbot's home directory:
   ```bash
    cd ~
    git clone your_url
    ```
3. Compile the code
  ```bash
  cd ~/mbot_firmware_ros
  mkdir build
  cd build
  cmake ..
  make
  ```
  - This is going to take a while.
4. After it's done, flash the compiled uf2 files
  ```bash
  sudo mbot-upload-firmware flash mbot_classic_ros.uf2
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
Open one terminal with Minicom to monitor the calibration output, and use another terminal to flash the calibration firmware to the Pico. Run the calibration several times and collect the resulting data for analysis. This task focuses on analyzing the system’s performance.

```bash
# terminal 1
sudo minicom -D /dev/ttyACM0 -b 115200
# terminal 2
cd ~/mbot_firmware_ros/build
sudo mbot-upload-firmware flash mbot_calibrate_classic.uf2 
```

Once you’ve finished collecting data, remember to flash **the firmware** back to the control board.
```bash
cd ~/mbot_firmware_ros/build
sudo mbot-upload-firmware flash mbot_classic_ros.uf2
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

We provide two Python scripts for testing:
1. `mbot_firmware_ros/python-tests/test_wheel_pid.py`: This script drives the robot and prints the target vs. actual wheel speeds in the terminal. Use it as a starting point, modify it to drive longer or more complex paths, then use it to make comparisons and collect data for your plots.
2. `mbot_firmware_ros/python-tests/drive_square.py`: This script drives the robot in a square pattern. Use the taped square on the lab floor and check whether your robot can follow the square and return to the starting point.
  - Note that every robot is slightly different in size. Not only are the PID gains different for each robot, but so are the hardware parameters. 
  - Go to `~/mbot_firmware_ros/include/config` and open `mbot_classic_config.h`. You may adjust the `DIFF_BASE_RADIUS` value (in meter), this value represents the **robot’s base radius**. The **base diameter** is the distance from the center of one wheel to the center of the other. If your robot isn’t driving in a proper square, this parameter may be inaccurate.
  - If you modified the **robot’s base radius**, remember to recompile the firmware and flash it to the control board.
    ```bash
      cd ~/mbot_firmware_ros/build
      cmake ..
      make
      sudo mbot-upload-firmware flash mbot_classic_ros.uf2
    ```

**Tip #1**: You don’t have to make all controllers perfect. The required plots are only for comparison, to show how the PID controller improves performance.

**Tip #2**: If the robot doesn’t drive a perfect square, it also might not be your controller, it could be due to inaccurate odometry. Continue to Task 1.3 to explore ways to improve it further.


{: .required_for_report }
Plot of time vs. velocity with robot responding to a step command of 0.5 m/s for the FF model, PID controller model and FF + PID controller (3 traces on one plot).
<br><br> Questions to Consider:
<br> 1) Which wheel controller performs the best and the worst, why?
<br> 2) Is there any improvement we can make?


## Task 1.3 Improve firmware (optional)

Odometry estimates your robot’s position and orientation by integrating wheel encoder measurements over time. In the firmware code `~/mbot_firmware_ros/src/mbot_odometry.c`, we currently use a simple dead-reckoning approach, which may not provide the most accurate results.

While the PID controllers help maintain accurate wheel speeds, you can add further enhancements to improve overall motion performance.

In this section, you’ll have the opportunity to enhance the firmware following the provided guide. **This task is optional**, you can choose to improve it for future challenges, or skip Task 1.3 entirely if your robot already performs well enough for your needs.

### TODO
1. Use `~/mbot_firmware_ros/src/mbot_odometry.c` as a starting point to understand how odometry is calculated, and consider adding improvements. For example, you could incorporate gyro readings and implement gyrodometry for better accuracy.
2. Read `~/mbot_firmware_ros/src/mbot_classic_ros.c`. This is the source code of the firmware uf2 file you compiled. The PID controller implementation starts around line 309, if you want to add features such as filters, this is a good place to begin.

**How to test your changes?**
1. Compile your code and flash it to the control board:
  ```bash
  cd ~/mbot_firmware_ros/build
  cmake ..
  make
  sudo mbot-upload-firmware flash mbot_classic_ros.uf2
  ```
2. Use teleop to drive the robot over a known distance and angle, then check your odometry values:
  ```bash
  ros2 topic echo /odom --field pose
  ```
3. Run the `mbot_firmware_ros/python-tests/drive_square.py` script and see if the robot’s performance has improved.

{: .required_for_report }
Describe what you added to the firmware and explain your reasoning.

## Task 1.4 Motion Controller
The motion controller takes a series of waypoints as input and generates velocity commands as output to navigate through them.

In this task, you will implement a rotate-translate-rotate controller. This is a high-level strategy that uses a PID controller to precisely execute each movement: rotating to the goal, driving forward, and rotating to the final orientation.

The PID controller in the firmware is used to control the wheel speed, while the PID controller in this task is used to control the traveled distance.

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
3. Implement the controller in the file `mbot_setpoint/src/motion_controller_diff.cpp`.
You can search for the keyword “TODO”, all the functions you need to complete are marked with “TODO” and numbered. Follow the numbering sequence as you implement them.
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


**Tip:** You don’t have to strictly follow the provided TODOs to fill up the blanks, if you want to implement a more sophisticated controller, feel free to do so. The template is intended to make things easier, not necessarily to achieve the best possible performance.

**How to test?**
1. Run the motion controller
  ```bash
  ros2 run mbot_setpoint motion_controller_diff
  ```
2. Publish the waypoints. By default, this node publishes a square with 1-meter sides:
  ```bash
  ros2 run mbot_setpoint square_publisher
  ```
3. The motion controller node will log values and save them in the `logs` directory. Once the square-driving task is complete, you can run the Python script to plot the data. The generated images will also be saved in the `logs` directory.
  ```bash
  python3 motion_controller_plot.py
  ```
  - The log can help you tune the PID gains, and the logging code in the controller program can also be used as a starting point for the required plots.

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