---
layout: default
title: Checkpoint 3
nav_order: 4
parent: Checkpoints
grand_parent: Botlab
last_modified_at: 2025-12-15 19:09:00 -0500
---


Using the SLAM algorithm you implemented previously, you can now construct a map of an environment with the MBot. In this checkpoint, you will add path planning and autonomous exploration capabilities.

We also provide a guide on [how to use slam_toolbox](/docs/botlab/how-to-guide/slam-toolbox-guide)! If you’re not satisfied with your mapping performance and need a map to test your A* or exploration algorithms, feel free to take a look.

You may also use slam_toolbox for mapping in Competition Event 2 and Event 3 (with point deductions). For details, please check the competition page.

### Contents
* TOC
{:toc}

## Task 3.1 Path Planning
 
Write an A* path planner. The A* skeleton is provided in the `mbot_nav` package.

### TODO
1. Check the [mbot_ros_labs](https://gitlab.eecs.umich.edu/rob550-f25/mbot_labs_ws) upstream for any new commits to pull.
2. All work for this task is in the package `mbot_nav`. Start with `navigation_node.cpp`, search for TODOs. All the actual code writing is in `astar.cpp`.
   - You also need to complete `obstacle_distance_grid.cpp`. The TODOs match the earlier tasks, so you can reuse your previous implementations.
   - You don’t need to follow the TODOs strictly, feel free to implement them in your own preferred way.
3. When finished, compile your code:
    ```bash
    cd ~/mbot_ros_labs
    colcon build --packages-select mbot_nav
    source install/setup.bash
    ```
   - {: .text-red-200} **Important: You must source the workspace in every relevant terminal after each build. If you don’t, ROS will keep using the old code, and your changes will not take effect.**
  
**How to test?**
- **Testing Mode**: In this mode, the navigation node listens to `/initialpose` and `/goal_pose` topics. Setting both in RViz triggers the A* planner, and the planned path will be displayed in RViz if successful. No real robot movement occurs, this is purely a software path calculation. Start with Testing Mode to verify that your A* algorithm works correctly.
   1. Run launch file to publish map and run nagivation node in **VSCode Terminal**:
      ```bash
      ros2 launch mbot_nav path_planning.launch.py map_name:=maze1
      ```
   2. Open Rviz to set initial pose and goal pose in **NoMachine Terminal**:
      ```bash
      cd ~/mbot_ros_labs/src/mbot_nav/rviz
      ros2 run rviz2 rviz2 -d path_planning.rviz
      ```

   **Video Demo**

   <iframe width="560" height="315" src="https://www.youtube.com/embed/6DEOXMysMj0?si=vDEOTjKrgOOb5TID" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

- **Real-world mode (with localization)**: After validating your planner in the previous tests, run in the real maze.
   1. Construct a map and save it in `mbot_ros_labs/src/mbot_nav/maps`. You may use your own mapping code or slam_toolbox.
   2. Then compile the `mbot_nav` package:
      ```bash
      cd ~/mbot_ros_labs
      colcon build --packages-select mbot_nav
      source install/setup.bash
      ```
   3. Launch the robot model, TF, LiDAR node in **VSCode Terminal #1**.
      ```bash
      ros2 launch mbot_bringup mbot_bringup.launch.py 
      ```
   4. Run launch file to publish map and run nagivation node in **VSCode Terminal #2**:
      ```bash
      ros2 launch mbot_nav path_planning.launch.py map_name:=your_map pose_source:=tf
      ```
   5. Run localization node in **VSCode Terminal #3**:
      ```bash
      ros2 run mbot_localization localization_node
      ```
   6. Start rviz and set initial pose in **NoMachine Terminal #1**, localization node needs it to initialize particles.
      ```bash
      cd ~/mbot_ros_labs/src/mbot_nav/rviz
      ros2 run rviz2 rviz2 -d path_planning.rviz
      ```
   7. Run motion controller in **VSCode Terminal #4**:
      ```bash
      ros2 run mbot_setpoint motion_controller_diff --ros-args -p use_localization:=true
      ```
   8. Then set the goal pose on rviz.

   **Video Demo**

   Some of the website content shown in the video may look different. Please follow the current version of the website, the video is only meant to demonstrate how to set the pose in RViz and what the expected results should look like.

   <iframe width="560" height="315" src="https://www.youtube.com/embed/tvFl81xeLKM?si=2kD8L4bCE4BTx1s-" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

{: .required_for_report } 
Provide a figure showing the planned path in the map.


## Task 3.2 Map Exploration

Until now, the MBot has only moved using teleop commands or manually set goal poses. For this task, you will implement a frontier-based exploration algorithm that allows the MBot to autonomously select targets and explore the full environment.

This task is useful for competition but **not required for Checkpoint 3 submission**.

### TODO
1. All work is in `mbot_nav`. Start with `exploration_node.cpp`, search for TODOs. All the actual code writing is in `frontier_explorer.cpp`.
   - You don’t need to follow the TODOs strictly, feel free to implement them in your own preferred way.
2. When finished, compile your code:
    ```bash
    cd ~/mbot_ros_labs
    colcon build --packages-select mbot_nav
    source install/setup.bash
    ```
   - {: .text-red-200} **Important: You must source the workspace in every relevant terminal after each build. If you don’t, ROS will keep using the old code, and your changes will not take effect.**
  
**How to test?**
1. Start rviz in **NoMachine Terminal #1**:
   ```bash
   cd ~/mbot_ros_labs/src/mbot_nav/rviz
   ros2 run rviz2 rviz2 -d path_planning.rviz
   ```
2. Launch the robot model, TF, LiDAR node in **VSCode Terminal #1**.
   ```bash
   ros2 launch mbot_bringup mbot_bringup.launch.py 
   ```
3. Run slam in **VSCode Terminal #2**:
   ```bash
   ros2 run mbot_slam slam_node
   ```
4. Run motion controller in **VSCode Terminal #3**:
   ```bash
   ros2 run mbot_setpoint motion_controller_diff --ros-args -p use_localization:=true
   ```
5. Run the exploration node in **VSCode Terminal #4**:
   ```bash
   ros2 run mbot_nav exploration_node
   ```

{: .required_for_report } 
Explain the strategy used for finding frontiers and any other details about your implementation that you found important for making your algorithm work.

## Task 3.3 Localization with Estimated and Unknown Starting Position

For advanced competition levels, the MBot must localize itself in a known map without knowing its initial pose. This will require initializing your particles in some distribution in open space on the map, and converging on a pose. This is useful in the competition but **does not required any submission in the Checkpoint 3**.

Details please check competition event 2 - level 3.

{: .required_for_report } 
Explain the methods used for initial localization.

## Checkpoint Submission

Demonstrate your path planner (task 3.1) by showing your robot navigating a maze.
- Submit a video of your robot autonomously navigating in a maze environment.
   - Your video should include the following:
      - Set goal pose, then the robot driving in the real-world lab maze.
      - Your visualization tool (RViz or Foxglove) displaying the map and planned path.
