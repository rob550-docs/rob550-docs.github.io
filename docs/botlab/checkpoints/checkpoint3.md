---
layout: default
title: Checkpoint 3
nav_order: 5
parent: Checkpoints
grand_parent: Botlab
last_modified_at: 2024-01-02 16:37:48 -0500
---

Using the SLAM algorithm implemented previously, you can now construct a map of an environment using the MBot simulator. You will implement additional capabilities for the MBot: path planning using A* and autonomous exploration of an environment.

### Contents
* TOC
{:toc}

## Task 3.1 Obstacle Distance
 
The robot configuration space is implemented in `mbot/mbot_autonomy/src/planning/obstacle_distance_grid.h/.cpp`. Use obstacle_distance_grid_test to check if your code passes the three tests, depending on your implementation, and data representation, you may want to modify the test as well. The botgui already has a call for a mapper object from which it can obtain and draw an obstacle distance grid. You can view the generated obstacle distance grid by ticking “Show Obstacle Distances” in botgui.

## Task 3.2 A* Path Planning
 
Write an A* path planner that will allow you to find plans from one pose in the environment to  another. You will integrate this path planner with the motion_controller from checkpoint 1 to allow your MBot to navigate autonomously through the environment.

For this phase, you will implement an A* planner and a simple configuration space for the MBot. The skeleton for the A* planner is located in `mbot/mbot_autonomy/src/planning/astar.h/.cpp` Your implementation will be called by the code in `mbot/mbot_autonomy/src/planning/motion_planner.h/.cpp`. You can test your astar_test code using botgui, check astar_test code for appropriate arguments.

Once your planner is implemented, test it by constructing a map using your SLAM algorithm and then using botgui to generate a plan. Right-click somewhere in free space on your map. Your planner will then be run inside botgui and a robot_path_t will be sent to the motion_controller for execution.

astar_test & astar_test_files can be used to test the performance of your A* planner.

Required For the Report:
- Provide a figure showing the planned path in an environment of your creation with the actual path driven by your robot overlayed on top. 
- Using astar_test_files, report statistics on your path planning execution times for each of the example problems in the data/astar folder -- you simply need to run astar_test_files after implementing your algorithm. If your algorithm is optimal and fast, great. If not please discuss possible reasons and strategies for improvement.

## Task 3.3 Map Exploration

Up to now, your MBot has always been driven by hand or driven to goals selected by you. For this task, you’ll write an exploration algorithm that will have the MBot autonomously select its motion targets and plan and follow a series of paths to fully explore an environment.

We have provided an algorithm for finding the frontiers -- the borders between free space and unexplored space -- in `mbot/mbot_autonomy/src/planning/frontiers.h/.cpp`. 

Plan and execute a path to drive to the frontier. Continue driving until the map is fully explored, i.e. no frontiers exist. Once you have finished exploring the map, return to the starting position. Your robot needs to be within 0.05m of the starting position. The state machine controlling the exploration process is located in `mbot/mbot_autonomy/src/planning/exploration.h/.cpp`.

Required For the Report: 
- Explain the strategy used for finding frontiers and any other details about your implementation that you found important for making your algorithm work.

## Task 3.4 Map Localization with Estimated and Unknown Starting Position:

For the advanced competition tasks, you will be asked to localize on a map where you do not know, or must estimate your initial position. This will require initializing your particles in some distribution in open space on the map, and converging on a pose.

Required for report:  
- Explain the methods used for initial localization.