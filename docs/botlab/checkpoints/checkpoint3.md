---
layout: default
title: Checkpoint 3
nav_order: 4
parent: Checkpoints
grand_parent: Botlab
last_modified_at: 2025-11-16 11:58:00 -0500
---

Using the SLAM algorithm you implemented previously, you can now construct a map of an environment with the MBot. In this checkpoint, you will add path planning and autonomous exploration capabilities.

### Contents
* TOC
{:toc}

## Task 3.1 Path Planning
 
Write an A* path planner. The A* skeleton is provided in the `mbot_nav` package.

### TODO
1. Pull the latest code from [mbot_ros_labs](https://gitlab.eecs.umich.edu/rob550-f25/mbot_labs_ws) upstream to get the `mbot_nav`.
2. All work for this task is in the package `mbot_nav`.
   - Start with `navigation_node.cpp`, search for TODOs. All the actual code writing is in `astar.cpp`.
   - You also need to complete `obstacle_distance_grid.cpp`. The TODO matches earlier tasks, so you can copy/paste your previous implementation. A new function `getOccupancy` has been added.
   - You don’t need to follow the TODOs strictly, feel free to implement them in your own preferred way.
3. When finished, compile your code:
    ```bash
    cd ~/mbot_ros_labs
    colcon build --packages-select mbot_nav
    source install/setup.bash
    ```

**How to test?**
- **Unit test**: This test will simply test if the code can find a valid path.
   ```bash
   ros2 run mbot_nav astar_test
   ```
- **Testing Mode**: this mode, the navigation node listens to `/initialpose` and `/goal_pose`. Setting both in RViz triggers the A* planner, the planned path will appear in RViz if successful.
   1. Run launch file to publish map and run nagivation node in **VSCode Terminal**:
      ```bash
      ros2 launch mbot_nav path_planning.launch.py map_name:=maze1
      ```
   2. Open Rviz to set initial pose and goal pose in **NoMachine Terminal**:
      ```bash
      cd ~/mbot_ros_labs/src/mbot_nav/rviz
      ros2 run rviz2 rviz2 -d path_planning.rviz
      ```
- **Real-world mode (with localization)**: After validating your planner in the previous tests, run in the real maze.
   1. Construct a map and save it in `mbot_ros_labs/src/mbot_nav/maps`. Then compile the `mbot_nav` package:
      ```bash
      cd ~/mbot_ros_labs
      colcon build --packages-select mbot_nav
      source install/setup.bash
      ```
   2. Run launch file to publish map and run nagivation node in **VSCode Terminal #1**:
      ```bash
      ros2 launch mbot_nav path_planning.launch.py map_name:=your_map pose_source:=tf
      ```
   3. Run localization node in **VSCode Terminal #2**:
      ```bash
      ros2 run mbot_localization localization_node
      ```
      - **Notice**: In `localization_node.cpp`, set `publish_map_odom_{true}` for real-world operation. Then **recompile the `mbot_slam`**.
      {: .text-red-200}
   4. Start rviz and set initial pose in **NoMachine Terminal #1**, localization node needs it to initialize particles.
      ```bash
      cd ~/mbot_ros_labs/src/mbot_nav/rviz
      ros2 run rviz2 rviz2 -d path_planning.rviz
      ```
   5. Run motion controller in **VSCode Terminal #3**:
      ```bash
      ros2 run mbot_setpoint motion_controller_diff
      ```
   6. Then set the goal pose on rviz.


You may also test using rosbag playback. This is useful for A* debugging but does not reflect real-world localization performance. Instructions for rosbag testing are shown in the video demo.
```bash
ros2 run mbot_nav navigation_node --ros-args -p pose_source:=tf
```
```bash
cd ~/mbot_ros_labs/src/mbot_rosbags/maze1
ros2 bag play maze1.mcap
```

### Video Demo

Video will be released soon.
{: .text-red-200}

{: .required_for_report } 
Provide a figure showing the planned path in the map.


## Task 3.2 Map Exploration

Until now, the MBot has only moved using teleop commands or manually set goal poses. For this task, you will implement a frontier-based exploration algorithm that allows the MBot to autonomously select targets and explore the full environment.

This task is useful for competition but **not required for Checkpoint 3 submission**.

### TODO
1. All work is in `mbot_nav`.
   - Start with `exploration_node.cpp`, search for TODOs. All the actual code writing is in `frontier_explorer.cpp`.
   - You don’t need to follow the TODOs strictly, feel free to implement them in your own preferred way.
2. When finished, compile your code:
    ```bash
    cd ~/mbot_ros_labs
    colcon build --packages-select mbot_nav
    source install/setup.bash
    ```

**How to test?**
1. Start rviz in **NoMachine Terminal #1**:
   ```bash
   cd ~/mbot_ros_labs/src/mbot_nav/rviz
   ros2 run rviz2 rviz2 -d path_planning.rviz
   ```
2. Run slam in **VSCode Terminal #1**:
   ```bash
   ros2 run mbot_slam slam_node
   ```
3. Run motion controller in **VSCode Terminal #2**:
   ```bash
   ros2 run mbot_setpoint motion_controller_diff
   ```
4. Run the exploration node in **VSCode Terminal #3**:
   ```bash
   ros2 run mbot_nav exploration_node
   ```

You may also test with rosbag playback. This is useful for algorithm debugging but does not represent true performance with real motion control. Instructions for rosbag testing are shown in the video demo.
```bash
# first
ros2 run mbot_nav exploration_node
# then 
ros2 run mbot_slam slam_node
```
```bash
cd ~/mbot_ros_labs/src/mbot_rosbags
ros2 bag play slam_test
```

### Video Demo
Video will be released soon.
{: .text-red-200}

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