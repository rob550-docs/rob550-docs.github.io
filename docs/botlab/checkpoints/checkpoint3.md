---
layout: default
title: Checkpoint 3
nav_order: 4
parent: Checkpoints
grand_parent: Botlab
last_modified_at: 2025-04-15 1:00:00 -0500
---

This checkpoint still under editing
{: .fs-5 .text-red-200 .fw-500}

Using the SLAM algorithm implemented previously, you can now construct a map of an environment using the MBot. You will implement additional capabilities for the MBot: path planning using A* and autonomous exploration of an environment.

### Contents
* TOC
{:toc}

## Task 3.1 Obstacle Distance

The robot configuration space is implemented in `mbot_autonomy/src/planning/obstacle_distance_grid.cpp|hpp`. An 8-connected Brushfire algorithm implementation is already written for you. If you make any changes, you can use obstacle_distance_grid_test.cpp to check if your code passes the three tests. Depending on your implementation and data representation, you may want to modify the tests as well. The botgui already has a call for a mapper object from which it can obtain and draw an obstacle distance grid. ~~You can view the generated obstacle distance grid by ticking “Show Obstacle Distances” in botgui.~~ This doesn't seem to work now :(

## Task 3.2 A* Path Planning
 
Write an A* path planner that will allow you to find plans from one pose in the environment to  another. You will integrate this path planner with the motion_controller from checkpoint 1 to allow your MBot to navigate autonomously through the environment.

For this phase, you will implement an A* planner and a simple configuration space for the MBot. The skeleton for the A* planner is located in `mbot_autonomy/src/planning/astar.cpp|hpp` Your implementation will be called by the code in `mbot_autonomy/src/planning/motion_planner.cpp|hpp`. You can test your astar_test code using botgui, check the astar_test code for the appropriate arguments. The astar_test program can be used to test the performance of your A* planner.

Your A* implementation may fail on a certain test case in the narrow constriction grid. This happens because the narrow constriction is on the edge of what is safe and what is unsafe. To fix this, you can multiply the `params.minDistanceToObstacle` variable by 1.01 when you are checking if something is safe inside `astar.cpp`.

Once your planner is implemented, test it in a real maze by constructing a map using your SLAM algorithm and then using botgui to generate a plan. The full process to testing your A* in a real maze is:

1. Start botgui
2. Run the following executables:
   - mbot_motion_controller
   - mbot_slam
   - motion_planning_server
3. Make a map of the maze, likely by driving around manually
4. In botgui, right click on an empty cell far away from the walls to plan a path to that cell

{: .required_for_report } 
1) Provide a figure showing the planned path in an environment of your creation with the actual path driven by your robot overlayed on top. <br>
<br> 2) Using astar_test, report statistics on your path planning execution times for each of the example problems in the mbot_example_logs/astar folder -- you simply need to run astar_test after implementing your algorithm. If your algorithm is optimal and fast, great. If not please discuss possible reasons and strategies for improvement.

## Task 3.3 Map Exploration

Up to now, your MBot has always been driven by hand or driven to goals selected by you. For this task, you’ll write an exploration algorithm that will have the MBot autonomously select its motion targets and plan and follow a series of paths to fully explore an environment. This is useful in the competition but is **not used in the Checkpoint 3 submission**.

We have provided an algorithm for finding the frontiers -- the borders between free space and unexplored space -- in `mbot_autonomy/src/planning/frontiers.cpp|hpp`. 

Plan and execute a path to drive to the frontier. Continue driving until the map is fully explored, i.e. no frontiers exist. Once you have finished exploring the map, return to the starting position. Your robot needs to be within 0.05m of the starting position. The state machine controlling the exploration process is located in `mbot_autonomy/src/planning/exploration.cpp|hpp`.

{: .required_for_report } 
Explain the strategy used for finding frontiers and any other details about your implementation that you found important for making your algorithm work.

## Task 3.4 Map Localization with Estimated and Unknown Starting Position:

For the advanced competition tasks, you will be asked to localize on a map where you do not know, or must estimate your initial position. This will require initializing your particles in some distribution in open space on the map, and converging on a pose. This is useful in the competition but is **not used in the Checkpoint 3 submission**.

{: .required_for_report } 
Explain the methods used for initial localization.

## Checkpoint Submission (Due 4/17/25)

Demonstrate your path planner and show your bot moving in a maze while avoiding obstacles.

- Submit a video of your bot navigating autonomously in a maze environment.
- Your video should include the following elements:
  - Show your bot in a maze
  - Show your map on botgui
  - Right click on botgui so a green path shows up from your bot’s current location to the clicked location
  - Show your bot moving in the maze to the point you clicked
