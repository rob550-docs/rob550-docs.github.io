---
layout: default
title: slam_toolbox Guide
parent: How-to Guide
grand_parent: Botlab
nav_order: 6
last_modified_at: 2025-11-19 15:37:48 -0500
---

Slam Toolbox is a set of tools and capabilities for 2D SLAM. More details can be found on its [GitHub page](https://github.com/SteveMacenski/slam_toolbox).

In this guide, we list all the commands you’ll need for tasks related to the final competition. Basically whenever you would normally run mbot_slam, simply replace those commands with the corresponding slam_toolbox commands.

### Contents
* TOC
{:toc}

## Teleop mapping

1. **VSCode Terminal #1**: launch the robot model, TF, LiDAR node.
    ```bash
    ros2 launch mbot_bringup mbot_bringup.launch.py 
    ```
2. Put the robot in the maze
3. **VSCode Terminal #2**: launch slam_toolbox
    ```bash
    ros2 launch mbot_navigation slam_toolbox_online_async_launch.py
    ```
4. **VSCode Terminal #3**: Run teleop node to drive robot to explore the area
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
5. Start rviz or foxglove
    ```bash
    # NoMachine
    cd ~/mbot_ws/src/mbot_navigation/rviz
    ros2 run rviz2 rviz2 -d mapping.rviz
    ```
    ```bash
    # VSCode Terminal #4
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml
    ```
6. **VSCode Terminal #5**: Save the map. In this example, we save it to the `mbot_nav/maps` folder so it can be used for A* path-planning tests.
    ```bash
    cd ~/mbot_ros_labs/src/mbot_nav/maps
    ros2 run nav2_map_server map_saver_cli -f map_name
    ```

## Goal setting exploration

1. **VSCode Terminal #1**: launch the robot model, TF, LiDAR node.
    ```bash
    ros2 launch mbot_bringup mbot_bringup.launch.py 
    ```
2. Put the robot in the maze
3. **VSCode Terminal #2**: launch slam_toolbox
    ```bash
    ros2 launch mbot_navigation slam_toolbox_online_async_launch.py
    ```
4. **VSCode Terminal #3**: Run motion controller, notice, this is the controller in `mbot_nav`.
    ```bash
    ros2 run mbot_nav motion_controller_diff
    ```
5. **VSCode Terminal #4**: Run path planner
    ```bash
    ros2 run mbot_nav navigation_node --ros-args -p pose_source:=tf
    ```
6. Start rviz. You can use "2D Goal Pose" to set the goal pose to lead the mbot to explore.
    ```bash
    # NoMachine
    cd ~/mbot_ros_labs/src/mbot_nav/rviz
    ros2 run rviz2 rviz2 -d path_planning.rviz
    ```
7. **VSCode Terminal #6**: Save the map
    ```bash
    cd ~/mbot_ros_labs/src/mbot_nav/maps
    ros2 run nav2_map_server map_saver_cli -f map_name
    ```

## Autonomous Exploration
1. **VSCode Terminal #1**: launch the robot model, TF, LiDAR node.
    ```bash
    ros2 launch mbot_bringup mbot_bringup.launch.py 
    ```
2. Put the robot in the maze
3. **VSCode Terminal #2**: launch slam_toolbox
    ```bash
    ros2 launch mbot_navigation slam_toolbox_online_async_launch.py
    ```
4. **VSCode Terminal #3**: Run motion controller, notice, this is the controller in `mbot_nav`.
    ```bash
    ros2 run mbot_nav motion_controller_diff
    ```
5. **VSCode Terminal #4**: Run exploration node
    ```bash
    ros2 run mbot_nav exploration_node
    ```
6. Start rviz or foxglove
    ```bash
    # NoMachine
    cd ~/mbot_ws/src/mbot_navigation/rviz
    ros2 run rviz2 rviz2 -d mapping.rviz
    ```
    ```bash
    # VSCode Terminal #5
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml
    ```
7. **VSCode Terminal #6**: Save the map
    ```bash
    cd ~/mbot_ros_labs/src/mbot_nav/maps
    ros2 run nav2_map_server map_saver_cli -f map_name
    ```