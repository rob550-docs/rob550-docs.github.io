---
layout: default
title: Checkpoint 2
nav_order: 4
parent: Checkpoints
grand_parent: Botlab
last_modified_at: 2024-04-02 12:16:00 -0500
---

**Update 10/31/24:** Add command to clone mbot_example_logs

**Update 11/7/24:** Add instructions for Botgui

**Update 11/8/24:** Add instructions to test your SLAM

During the SLAM part of the lab, you will build an increasingly sophisticated algorithm for mapping and localizing in the environment. You will begin by constructing an occupancy grid using known poses. Following that, you’ll implement Monte Carlo Localization in a known map. Finally, you will put each of these pieces together to create a full SLAM system. Much of this can be initiated using only logs and the only parts you should need to change when you are ready to use the actual robot are the constants and parameters.


### Contents
* TOC
{:toc}

### Botgui
To do this section, make sure that mbot_gui is installed on your bot. If it isn't, go back to [Mbot System Setup](../mbot-system-setup-Pi5) and follow the instructions at the very end to install it.

Botgui is a legacy GUI and alternative to the webapp. It has more functionality compared to the webap, so it is useful for SLAM and beyond. When coding your SLAM, we highly recommend using botgui to visualize your map and pose.

To run it you can execute the following commands, but they must be executed **on NoMachine**! As botgui opens a new GUI, it will not work on SSH.

```bash
$ cd ~/mbot_ws/mbot_gui
$ source setenv.sh
$ ./build/botgui
```

### LCM Logs
For this phase, we have provided some LCM logs containing sensor data from the MBot along with ground-truth poses as estimated by the staff’s SLAM implementation.

The logs are in the [`mbot_example_logs`](https://gitlab.eecs.umich.edu/ROB550-F24/mbot_example_logs) repo, which you can clone to your bot with the following commands:

```bash
$ cd ~/mbot_ws
$ git clone https://gitlab.eecs.umich.edu/ROB550-F24/mbot_example_logs.git
```

Here are a few logs that will be helpful to tune your SLAM:

- `mbot_example_logs/botlab/center_maze.log`: convex environment, where all walls are always visible and the robot remains stationary (use for initial testing of algorithms).
- `mbot_example_logs/botlab/drive_square.log` : a convex environment while driving a square
- `mbot_example_logs/botlab/drive_maze.log`: driving a circuit in an environment with several obstacles

To play back these recorded LCM sessions, you can use lcm-logplayer-gui. Be careful, because the following command opens a new GUI and will only run **on NoMachine**!
```bash
$ lcm-logplayer-gui log_file.log
```
You can also run the command line version, lcm-logplayer, if the system does not have Java. In the GUI version you can turn off LCM channels with a checkbox. The same functionality is available in the command line version, check the help with:
```bash
$ lcm-logplayer --help
```
Similarly, to record your own lcm logs, for testing or otherwise, you can use the lcm-logger:
```bash
$ lcm-logger -c SLAM_POSE my_lcm_log.log 
```
This example would store data from the SLAM_POSE channel in the file `my_lcm_log.log`. With no channels specified, all channels will be recorded over the interval.  

{: .warning }
Whenever you test your slam using log files, make sure to unplug your lidar and Pico to prevent the realtime data interfering with the logged data.
 
### Accounting for Scan Speed
The laser rangefinder beam rotates 360 degrees at a sufficiently slow rate such that the robot may move a non-negligible distance in that time. Therefore, you will need to estimate the actual pose of the robot when each beam was sent to determine the cell in which the beam terminated. 

To do so, you must interpolate between the robot pose estimate of the current scan and the scan immediately before. Remember that the pose estimate of the previous SLAM update occurred immediately before the current scan started. Likewise, the pose estimate of the current SLAM update will occur when the final beam of the current scan is measured. We have provided code to handle this interpolation for you, located in `mbot_autonomy/src/slam/moving_laser_scan.hpp`. You only need to implement the poses to use for the interpolation. 

## Mapping - Occupancy Grid
### Task 2.1
Implement the occupancy grid mapping algorithm in the Mapping class in `mbot_autonomy/src/slam/mapping.cpp|hpp`.

An occupancy grid maps the space surrounding the robot by assigning probabilities to each cell, indicating whether they are occupied or free. In our case, we'll use a grid where cells are at most 10 cm in size, and their log-odds values range from -127 to 127, represented as integers. To accurately trace the rays across your occupancy grid, consider implementing Bresenham’s line algorithm, as discussed in our class lecture.

For this task, construct occupancy grid maps using the ground-truth poses provided in the log files. You can process these poses for map construction by enabling the `--mapping-only` command-line option when executing the slam program. This approach isolates the mapping function, using the specified poses from the log file.

{: .required_for_report } 
Plot your map from the log file `drive_maze.log`.


## Monte Carlo Localization
As discussed in the lecture, Monte Carlo Localization (MCL) is a particle-filter-based localization algorithm. Implementation of MCL requires three key components: 
- an action model to predict the robot’s pose
- a sensor model to calculate the likelihood of a pose given a sensor measurement
- various functions for particle filtering, including drawing samples, normalizing particle weights, and finding the weighted mean pose of the particles. 

In these tasks, you’ll run the slam program in localization-only mode using a saved map. Use the ground-truth maps provided with the sensor logs. You can run slam using: 
```bash
$ ./mbot_slam --localization-only --map <map_file.map>
```

To test the action model only, you can run in action-only mode:
```bash
$ ./mbot_slam --action-only
```

### Task 2.2.1 Action Model
Implement an action (or motion) model using some combination of sensors. This could be odometry with wheel encoders alone, or it could be some other estimate of action incorporating the IMU or using computer vision. The skeleton of the action model can be found in `mbot/mbot_autonomy/slam/action_model.cpp|hpp`.

Refer to Chapter 5 of Probabilistic Robotics for a discussion of common action models. You can base your implementation on the pseudo-code in this chapter. There are two action models that are discussed in detail, the Velocity Model (Sec. 5.3) and the Odometry Model (Sec. 5.4).

If you test in action-only mode you will see the particles spread out over time. That is expected. The action model adds randomness to your position, so your estimated position will get worse and worse if you don't use the sensor model to filter your position.

{: .required_for_report } 
Describe the action model you utilized, including the equations employed. Additionally, provide a table listing the values of any uncertainty parameters, and explain how you chose these values.

### Task 2.2.2 Sensor Model & Particle Filter
Implement a sensor model that calculates the likelihood of a pose given a laser scan and an occupancy grid map. The skeleton of the sensor model can be found in `mbot_autonomy/slam/sensor_model.cpp|hpp`.  

Refer to Chapter 6 of Probabilistic Robotics for a discussion of common sensor models. You can base your implementation on the pseudo-code in this chapter.  

Implement the particle filter functions in `mbot_autonomy/slam/particle_filter.cpp|hpp`.

Refer to Chapter 4 of Probabilistic Robotics for information implementing the particle filter.

Hint: In case of slow performance of your sensor model, consider increasing the ray stride in the MovingLaserScan call.

{: .required_for_report } 
1) Report in a table the time it takes to update the particle filter for 100, 500 and 1000 particles. Estimate the maximum number of particles your filter can support running at 10Hz.
<br> 2) Using your particle filter on `mbot_example_logs/botlab/drive_square.log`, plot 300 particles at regular intervals along the path taken.

## Simultaneous Localization and Mapping (SLAM)
### Task 2.3
You have now implemented mapping using known poses and localization using a known map. You can now run the slam program in SLAM mode, which uses the following simple SLAM algorithm:
- Use the first laser scan received to construct an initial map.
- For subsequent laser scans:
    - Localize using the map from the previous iteration.
    - Update the map using the pose estimate from your localization.

NOTE: The above logic is already implemented for you. Your goal for this task is to make sure your localization and mapping are accurate enough to provide a stable estimate of the robot pose and map. You will need good SLAM performance to succeed at the robot navigation task.

{: .required_for_report } 
1) Create a block diagram of how the SLAM system components interact
<br> 2) Compare the estimated poses from your SLAM system against the ground-truth poses in `drive_maze_full_rays.log`. Use this to estimate the accuracy of your system and include statistics such as RMS error etc.

## Testing Your SLAM

### Setup

Do these at the start of a session.

1. Connect to the bot using **NoMachine**
2. Unplug your Lidar and Pico.
3. Prepare a log file for playback using lcm-logplayer-gui.
    ```bash
    $ lcm-logplayer-gui log_file.log
    ```
    Disable the following channels depending on what you're testing:
    - Testing mapping: Disable SLAM_MAP
    - Testing anything else: Disable all SLAM_* channels

    Here is an image of what the GUI would look like if you're testing mapping:

<a class="image-link" href="/assets/images/botlab/checkpoints/logplayer_gui_mapping.png">
<img src="/assets/images/botlab/checkpoints/logplayer_gui_mapping.png" alt="This image shows the Lcm-Logplayer-GUI application playing the log file 'center_maze_full_rays.log'. The interface includes a play button, a step button, and a slider for navigating the log timeline. The slider is positioned at approximately 1.302 seconds into the log file. Below the timeline, a table lists various log channels and their corresponding playback channels. Each row represents a different data channel, with checkboxes in the 'Enable' column to toggle their playback status. The 'SLAM MAP' channel is highlighted, indicating it is currently selected but not enabled for playback." style="max-width:500px;"/>
</a>

### Running

Do these every time you run a new test.

1. Close botgui and kill your SLAM program if it's still running
2. Start botgui
3. Scrub the log player progress bar (the dark blue one at the top) slightly ahead of the start but not at the very start. See the image above for an example. This may not be necessary for every log file but playing from the very start can sometimes cause your SLAM to fail.
4. Run mbot_slam with a different command line argument depending on what you are testing:
    - Testing mapping:
        ```bash
        $ ./mbot_slam --mapping-only
        ```
    - Testing action model:
        ```bash
        $ ./mbot_slam --action-only
        ```
    - Testing localization:
        ```bash
        $ ./mbot_slam --localization-only --map <map_file.map>
        ```
    - Testing full SLAM:
        ```bash
        $ ./mbot_slam
        ```
5. Press "Play" on lcm-logplayer-gui.
6. Observe your SLAM results.
7. To compare your results with the ground truth results, stop your SLAM and play the log with the SLAM_* channels enabled.


## Checkpoint Submission
<br>
<a class="image-link" href="/assets/images/botlab/checkpoints/checkpoint1-maze.png">
<img src="/assets/images/botlab/checkpoints/checkpoint1-maze.png" alt=" " style="max-width:600px;"/>
</a>

Demonstrate your SLAM system by mapping the maze used for Checkpoint 1. You may either manually drive the path with teleop, click to drive in botgui, or try using your python script from checkpoint 1.
1. Submit a screenshot of the map on botgui, showing the SLAM_POSE and MBOT_ODOMETRY pose traces (turn off lasers, & frontiers)
2. Submit a lcm log file of the run and a .map file as well
    - You can uncomment the line map_.saveToFile("current.map"); in slam.cpp to output maps
    - You can also use the Download Map functionality of the webapp
4. Write a short description of your SLAM system (1/2 page) and any features we should be aware of.
