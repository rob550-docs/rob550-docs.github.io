---
layout: default
title: Checkpoint 2
nav_order: 3
parent: Checkpoints
grand_parent: Botlab
last_modified_at: 2025-12-15 19:09:00 -0500
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

There is a demo video below shows the workflow **after completing all TODOs in the template code**. It demonstrates how to map and how to view your map in RViz or Foxglove Studio.

### TODO
1. Check the [mbot_ros_labs](https://gitlab.eecs.umich.edu/rob550-f25/mbot_labs_ws) upstream for any new commits to pull.
2. Install Foxglove Bridge. We will introduce Foxglove Studio, a web-based visualization tool in this task, as an alternative to NoMachine.
    ```bash
    sudo apt install ros-$ROS_DISTRO-foxglove-bridge
    ```
3. All work for this task is in the package `mbot_mapping`. Start with `mapping_node.cpp`, search for TODOs.
4. When finished, compile your code:
    ```bash
    cd ~/mbot_ros_labs
    colcon build --packages-select mbot_mapping
    source install/setup.bash
    ```
   - {: .text-red-200} **Important: You must source the workspace in every relevant terminal after each build. If you don’t, ROS will keep using the old code, and your changes will not take effect.**


**Explanation of TODOs**

You have 3 major tasks in this task:
1. Lidar ray interpolation
    - A laser scan isn’t instantaneous, it takes time to complete. The robot is moving while the lidar is spinning, then:
        - The first beams are measured at start_pose.
        - The last beams are measured at end_pose.
    - We solve this in the function `MovingLaserScan::interpolateRay` in `moving_laser_scan.cpp`, which estimates the robot’s pose for each beam.
2. Bresenham’s algorithm
    - After interpolating all rays, use Bresenham’s algorithm to determine which cells in the occupancy grid each ray passes through.
3. Update map cells
    - For each beam, update the corresponding cells using log-odds to reflect occupancy probabilities.

For the Lidar ray interpolation part:
- According to the message definition of [sensor_msgs/LaserScan Message](https://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/LaserScan.html), "timestamp in the header is the acquisition time of the first ray in the scan." This means the odometry queried by the following line corresponds to the robot’s pose when the first LiDAR ray was fired:
    ```cpp
    geometry_msgs::msg::TransformStamped tf_odom_base = tf_buffer_->lookupTransform("odom", "base_footprint", scan->header.stamp);
    ```
- Thus, in the function `MovingLaserScan scan(*msg, current_pose, current_pose_end);`, which calls `interpolateRay(scan, start_pose, end_pose);`
  - The start pose should be estimate pose.
  - The end pose should be computed based on the scan duration and the robot’s motion.


**How to test?**
1. Run the following in **VSCode Terminal #1** to launch the lidar driver, robot tf tree, robot model:
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
4. Visualize the progress:
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
        cd ~/mbot_ros_labs
        # we need to re-compile so the launch file can find the newly saved map
        colcon build --packages-select mbot_mapping
        source install/setup.bash
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

There is a demo video below shows the workflow **after completing all TODOs in the template code**. It demonstrates how to play the ros bag and visualize localization progress in RViz or Foxglove Studio.

### TODO
1. All work for this task is in the package `mbot_localization`. Start with `localization_node.cpp`, search for TODOs.
2. When finished, compile your code:
    ```bash
    cd ~/mbot_ros_labs
    colcon build --packages-select mbot_localization
    source install/setup.bash
    ```
    - {: .text-red-200} **Important: You must source the workspace in every relevant terminal after each build. If you don’t, ROS will keep using the old code, and your changes will not take effect.**
  
**Explanation of TODOs**

Starting from `localization_node.cpp`, you have 2 main TODOs:
1. Construct the obstacle distance grid in `obstacle_distance_grid.cpp`. Pre-compute how far each cell is from the nearest obstacle. This will be used in the sensor model.
2. Complete the particle filter (more complex)
    1. Resample particles – resample particles based on their weights.
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
    ros2 run mbot_localization localization_node --ros-args -p publish_tf:=false
    ```
3. Play the ROS bag in the **VSCode Terminal #2**:
    ```bash
    cd ~/mbot_ros_labs/src/mbot_rosbags/maze1
    ros2 bag play maze1.mcap
    ```
    - **We provide a rosbag because** this task focuses on localization only, and we need a good map for that. Using the bag saves time and lets you work anywhere.
    - This ROS bag includes the `/cmd_vel` topic. You need to **unplug the Type-C cable from the Pi to the Pico** to prevent the robot from driving away.
    - All the data required for this task comes from the ROS bag, you **DO NOT** need to manually publish `/initialpose`, and **DO NOT** need to run `ros2 launch mbot_bringup mbot_bringup.launch.py`.
        ```bash
        # use the following command to check what is in the bag
        $ cd ~/mbot_ros_labs/src/mbot_rosbags
        $ ros2 bag info maze1
        ```
        - `/amcl_pose` serves as the reference pose, the "ground truth" you can use to compare against your estimated pose.

### Demo Video
<iframe width="560" height="315" src="https://www.youtube.com/embed/nieEYElK_Kc?si=OTV2XaRHv4rlvkNS" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

{: .required_for_report } 
1) Report in a table the time it takes to update the particle filter for 100, 500 and 1000 particles. Estimate the maximum number of particles your filter can support running at 10Hz.
<br> 2) Include a screenshot comparing your estimated pose path with the reference pose path.


## Task 2.3 Simultaneous Localization and Mapping (SLAM)
You have now implemented mapping using known poses and localization using a known map. You can now implement the following simple SLAM algorithm:
- Use the first laser scan received to construct an initial map.
- For subsequent laser scans:
    - Localize using the map from the previous iteration.
    - Update the map using the pose estimate from your localization.

### TODO
1. Check the [mbot_ros_labs](https://gitlab.eecs.umich.edu/rob550-f25/mbot_labs_ws) upstream for any new commits to pull.
2. All work for this task is in the package `mbot_slam`. Start with `slam_node.cpp`, search for TODOs. 
    - Most TODOs come from Task 2.1 and Task 2.2. You can simply copy and paste your code from those tasks, tune the parameters, and achieve a good working SLAM system. You don’t need to follow the TODOs strictly, feel free to implement them in your own preferred way.
3. When finished, compile your code:
    ```bash
    cd ~/mbot_ros_labs
    colcon build --packages-select mbot_slam
    source install/setup.bash
    ```
   - {: .text-red-200} **Important: You must source the workspace in every relevant terminal after each build. If you don’t, ROS will keep using the old code, and your changes will not take effect.**

**Note: The LiDAR is physically mounted backward**, resulting in a 3.14 rad yaw offset between the base_footprint frame and the lidar frame. When projecting LiDAR data into the world frame, be sure to account for this offset.
- This offset is defined in the URDF file located at: `~/mbot_ws/src/mbot_description/urdf`
- The TF tree is structured as follows: `map → odom → base_footprint → base_link → lidar_link`. When you query the transform `odom → base_footprint`, the system does not include the relationship between `base_footprint → lidar_link`, we have to manually add the offset. When you query `odom → lidar_link`, the yaw offset is already included, so no manual adjustment is needed.

---
**Hints for parameter tuning:**

If your mapping and localization are working well, but your full SLAM system is struggling, the problem is often in parameter tuning. Here are some hints to guide your tuning:
1. Action model `action_model.hpp`
    - Observe your particle cloud: What is its shape? Is it a tight circle, or an elongated oval following the robot's path? If `k2_` or `k1_` is too high, your particle cloud might be a "long oval".
    - We know the robot's odometry isn't perfect, but it's also not completely wrong. Is a large noise value like 1.0 necessary, or would a much smaller value like 0.001 be more appropriate? Think about how much you trust the odometry.
2. Sensor model `sensor_model.hpp`
    - `sigma_hit_` is in meters. If your map resolution is 0.05m (5cm) and your `sigma_hit_` is also 0.05m, what does that imply? It means a lidar ray landing one cell away from the obstacle in the map is already at one standard deviation of error.
    - A smaller `sigma_hit_` makes your sensor model stricter, demanding scans align almost perfectly. A larger value is more forgiving. Given the map resolution, experiment with values larger or smaller than 0.05m to see how it affects particle weights.
3. Mapping `mapping.hpp`
    - Do you see cells "flickering" between free and occupied? This can happen if a few bad scans can erase a well-defined wall. Tune the log-odds limits: Consider tunning `kMaxLogOdds` and `kMinLogOdds`. to make it harder for a cell's state to be "flipped" after it's been confidently marked as occupied or free.
4. Map resolution `slam_node.cpp`
    - High-resolution maps help localization. Your Task 2.2 may have worked well because the provided map was "crispy" (e.g., 0.01m resolution). If you are using a coarser resolution like 0.05m, try decreasing the cell size (e.g., to 0.03m or 0.02m) here `grid_(10.0, 10.0, 0.02, -5.0, -5.0)`.

Here are examples of "OK," "Good," and "Excellent" maps generated from the slam_test bag. These are related to how competition scoring will be evaluated.
<div class="popup-gallery">
    <a href="/assets/images/botlab/checkpoints/checkpoint2-ok-map.png" title=""><img src="/assets/images/botlab/checkpoints/checkpoint2-ok-map.png" width="200" height="200"></a>
    <a href="/assets/images/botlab/checkpoints/checkpoint2-good-map.png" title=""><img src="/assets/images/botlab/checkpoints/checkpoint2-good-map.png" width="220" height="200"></a>
    <a href="/assets/images/botlab/checkpoints/checkpoint2-excellent-map.png" title=""><img src="/assets/images/botlab/checkpoints/checkpoint2-excellent-map.png" width="225" height="200"></a>
</div>

---

**How to test?**
1. Run the following in **VSCode Terminal #1**:
    ```bash
    ros2 launch mbot_bringup mbot_bringup.launch.py
    ```
2. Put the robot in the maze and start the SLAM Node in **VSCode Terminal #2**:
    ```bash
    ros2 run mbot_slam slam_node
    ```
3. Start Rviz **on NoMachine**:
    ```bash
    cd ~/mbot_ros_labs/src/mbot_slam/rviz
    ros2 run rviz2 rviz2 -d slam.rviz
    ```
4. Map in the lab maze, drive the robot manually using the teleop in **VSCode Terminal #3**:
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
    - Alternatively, you can work from anywhere using the provided rosbag:
    ```bash
    cd ~/mbot_ros_labs/src/mbot_rosbags
    ros2 bag play slam_test
    ```
        - This ROS bag includes the `/cmd_vel` topic. Unplug the Type-C cable from the Pi to the Pico before run the ros bag.
5. After mapping the whole area, **do not stop the mapping node yet**. Save the map in **VSCode Terminal #4**:
    ```bash
    cd ~/mbot_ros_labs/src/mbot_slam/maps
    ros2 run nav2_map_server map_saver_cli -f map_name
    ```
    - Replace `map_name` with the name you want for your map.

{: .required_for_report } 
1) Create a block diagram showing how the SLAM system components interact.
<br> 2) Include a screenshot of your map, and explain why your SLAM-generated map is better than the one from Task 2.1.

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
