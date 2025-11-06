---
layout: default
title: Checkpoint 2
nav_order: 3
parent: Checkpoints
grand_parent: Botlab
last_modified_at: 2025-11-03 19:52:48 -0500
---


During the SLAM part of the lab, you will build an increasingly sophisticated algorithm for mapping and localizing in the environment. You will begin by constructing an occupancy grid using known poses. Following that, you’ll implement Monte Carlo Localization in a known map. Finally, you will put each of these pieces together to create a full SLAM system.

### Contents
* TOC
{:toc}

## Design Lab

We’ve released the [Design Lab Guide](/docs/botlab/checkpoints/design-lab)!

You need to design and build a forklift mechanism. The design file is due with your Checkpoint 2 submission, but it doesn’t need to be final, a prototype version is perfectly fine. The goal is to help you stay on track.

Designing, prototyping, and testing should continue alongside the main checkpoints until the final competition.


## Task 2.1 Mapping

In this task, you will implement odometry-based occupancy grid mapping. 

**There is a demo video below shows the workflow after completing all TODOs in the template code. It demonstrates how to map and how to view your map in RViz or Foxglove Studio.**

### TODO
1. Pull the latest code from [mbot_ros_labs](https://gitlab.eecs.umich.edu/rob550-f25/mbot_labs_ws) upstream to get the Checkpoint 2 template.
2. Install Foxglove Bridge. We will introduce Foxglove Studio, a web-based visualization tool in this task, as an alternative to NoMachine.
    ```bash
    sudo apt install ros-$ROS_DISTRO-foxglove-bridge
    ```
3. All work for this task is in the package `mbot_mapping`.
    - Start with `mapping_node.cpp`, search for TODOs.
    - TODOs in each file are numbered in order. The order is how you should work within that file, not the overall task order across the project.
4. When finished, compile your code:
    ```bash
    cd ~/mbot_ros_labs
    colcon build --packages-select mbot_mapping
    source install/setup.bash
    ```

**Explanation of TODOs**

You have 3 major tasks in this task:
1. Lidar ray interpolation
    - A laser scan isn’t instantaneous, it takes time to complete. If the robot moves during the scan:
        - The first beams are measured at start_pose.
        - The last beams are measured at end_pose.
    - We solve this in the function interpolateRay(), which estimates the robot’s pose for each beam.
2. Bresenham’s algorithm
    - After interpolating all rays, use Bresenham’s algorithm to determine which cells in the occupancy grid each ray passes through.
3. Update map cells
    - For each beam, update the corresponding cells using log-odds to reflect occupancy probabilities.

**How to test?**
1. Run the following in **VSCode Terminal #1**:
    ```bash
    ros2 launch mbot_bringup mbot_bringup.launch.py
    ```
2. Run the following in the **VSCode Terminal #2** to start the mapping node:
    ```bash
    ros2 launch mbot_mapping mapping.launch.py
    ```
3. Using teleop control to move the robot in the maze, run in **VSCode Terminal #3**:
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
4. Visualize the Progress:
    - Option 1: In RViz (via NoMachine):
        ```bash
        rviz2
        ```
    - Option 2: In Foxglove Studio:
        ```bash
        # Start the ROS2-Foxglove bridge:
        ros2 launch foxglove_bridge foxglove_bridge_launch.xml
        ```
        - You can now visualize topics in Foxglove Studio.
5. After mapping the whole area, **do not stop the mapping node yet**. Save the map in **VSCode Terminal #4**:
    ```bash
    cd ~/mbot_ros_labs/src/mbot_mapping/maps
    ros2 run nav2_map_server map_saver_cli -f map_name
    ```
    - Replace `map_name` with the name you want for your map.
    - After saving, you can stop the mapping node.
6. To view the map:
    - Option 1: Use a local viewer. Install a VSCode extension like PGM Viewer to open your map file.
    - Option 2: Publish the map in ROS2
        ```bash
        ros2 launch mbot_mapping view_map.launch.py map_name:=map_name
        ```
        - This will publish the map to the `/map` topic.
        - You can now view it in RViz (via NoMachine) or Foxglove Studio.

**Expected Result:**
- If your map looks similar to the image below, that’s a good result, small errors are expected.

    <a class="image-link" href="/assets/images/botlab/checkpoints/checkpoint2-odom-mapping.png">
    <img src="/assets/images/botlab/checkpoints/checkpoint2-odom-mapping.png" alt=" " style="max-width:600px;"/>
    </a>

### Demo Video

<iframe width="560" height="315" src="https://www.youtube.com/embed/m2nrKbueu7U?si=IrZDUThvokpogPlR" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


{: .required_for_report } 
Include a screenshot of your map. Analyze the map quality and explain why it is not ideal.


## Task 2.2 Localization
Monte Carlo Localization (MCL) is a particle-filter-based localization algorithm. To implement MCL, you need three key components:
- Action model: predicts the robot’s next pose.
- Sensor model: calculates the likelihood of a pose given a sensor measurement.
- Particle filter functions: draw samples, normalize particle weights, and compute the weighted mean pose.

**There is a demo video below shows the workflow after completing all TODOs in the template code. It demonstrates how to play the ros bag and visualize localization progress in RViz or Foxglove Studio.**

### TODO
1. All work for this task is in the package `mbot_localization`.
    - Start with `localization_node.cpp`, search for TODOs.
    - TODOs in each file are numbered in order. The order is how you should work within that file, not the overall task order across the project.
2. When finished, compile your code:
    ```bash
    cd ~/mbot_ros_labs
    colcon build --packages-select mbot_localization
    source install/setup.bash
    ```

**Explanation of TODOs**

Starting from `localization_node.cpp`, you have 2 main TODOs:
1. Construct the obstacle distance grid. Pre-compute how far each cell is from the nearest obstacle. This will be used in the sensor model.
2. Complete the particle filter (more complex)
    1. Resample particles – select particles based on their weights.
    2. Apply the action model – move the particles according to odometry, adding noise.
    3. Apply the sensor model – update each particle’s weight based on how well its predicted sensor readings match the actual laser scan. Here we can use the pre-computed obstacle distance grid to calculate these likelihoods.
    4. Estimate the new pose – compute the weighted mean of the particles based on the posterior distribution.

**How to test?**
1. Run RViz **on NoMachine** using provided rviz config file:
    ```bash
    cd ~/mbot_ros_labs/src/mbot_localization/rviz
    ros2 run rviz2 rviz2 -d localization.rviz
    ```
2. Run the localization node in the **VSCode Terminal #1**:
    ```bash
    ros2 run mbot_localization localization_node
    ```
3. Play the ROS bag in the **VSCode Terminal #2**:
    ```bash
    cd ~/mbot_ros_labs/src/mbot_rosbags/maze1
    ros2 bag play maze1.mcap
    ```
    - The ROS bag publishes all the topics your node needs.


### Demo Video
<iframe width="560" height="315" src="https://www.youtube.com/embed/nieEYElK_Kc?si=OTV2XaRHv4rlvkNS" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

{: .required_for_report } 
1) Report in a table the time it takes to update the particle filter for 100, 500 and 1000 particles. Estimate the maximum number of particles your filter can support running at 10Hz.
<br> 2) Include a screenshot comparing your estimated pose path with the reference pose path.

---
Below this point is still under editing
{: .fs-5 .text-red-200 .fw-500}


## Task 2.3 Simultaneous Localization and Mapping (SLAM)
You have now implemented mapping using known poses and localization using a known map. You can now implement the following simple SLAM algorithm:
- Use the first laser scan received to construct an initial map.
- For subsequent laser scans:
    - Localize using the map from the previous iteration.
    - Update the map using the pose estimate from your localization.

NOTE: The above logic is already implemented for you. Your goal for this task is to make sure your localization and mapping are accurate enough to provide a stable estimate of the robot pose and map. You will need good SLAM performance to succeed at the robot navigation task.

{: .required_for_report } 
1) Create a block diagram of how the SLAM system components interact
<br> 2) Compare the estimated poses from your SLAM system against the reference poses in `maze.mcap`. Use this to estimate the accuracy of your system.


## Checkpoint Submission
<br>
<a class="image-link" href="/assets/images/botlab/checkpoints/checkpoint1-maze.png">
<img src="/assets/images/botlab/checkpoints/checkpoint1-maze.png" alt=" " style="max-width:600px;"/>
</a>

Demonstrate your SLAM system by mapping the maze used in Checkpoint 1. You may either manually drive the robot using teleop or use the motion controller from Checkpoint 1.
1. Submit a screenshot of the generated map.
2. Submit the map file itself.
3. Submit a short description of your SLAM system, including how it works and any key observations.
4. Submit your forklift design prototype, **it can be in any form, such as a simple sketch, a CAD file, or other representation.**