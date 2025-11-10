---
layout: default
title: Checkpoint 3
nav_order: 4
parent: Checkpoints
grand_parent: Botlab
last_modified_at: 2025-11-10 16:58:00 -0500
---

This checkpoint still under editing
{: .fs-5 .text-red-200 .fw-500}

Using the SLAM algorithm implemented previously, you can now construct a map of an environment using the MBot. In this checkpoint, you will implement additional capabilities for the MBot: path planning and autonomous exploration of an environment.

### Contents
* TOC
{:toc}

## Task 3.1 Path Planning
 
Write an A* path planner that will allow you to find plans from one pose in the environment to another. 

### TODO
For this phase, you will implement an A* planner and a simple configuration space for the MBot. The skeleton for the A* planner is located in `mbot_nav`.

We also provide test code.

Once your planner is implemented, test it in a real maze by constructing a map using your SLAM algorithm and then generate a plan. The full process to testing your A* in a real maze is:

1. Drive using teleop package to map and save the map.
2. Use rviz to set (initial pose) and goal pose
3. The robot should follow the waypoints to the goal pose.

{: .required_for_report } 
1) Provide a figure showing the planned path in the map with the actual path driven by your robot overlayed on top. <br>
<br> 2) Report statistics on your path planning execution times for each of the example ros bags in the `mbot_ros_bags/nav_test`. If your algorithm is optimal and fast, great. If not please discuss possible reasons and strategies for improvement.

## Task 3.2 Map Exploration

Up to now, your MBot has always been driven by teleop or driven to goal pose selected by you. For this task, you’ll write an exploration algorithm that will have the MBot autonomously select its motion targets and plan and follow a series of paths to fully explore an environment. This is useful in the competition but **does not required any submission in the Checkpoint 3**.

### TODO

implement fronter finding algorithm. Logic should be:
1. Running slam to map the environment
2. Finding frontiers
3. Planning a path to a frontier
4. Motion controller follow the waypoints to the frontier
5. Loop back the slam will map more enviroment and find new fontiers.

{: .required_for_report } 
Explain the strategy used for finding frontiers and any other details about your implementation that you found important for making your algorithm work.

## Task 3.3 Localization with Estimated and Unknown Starting Position

For the advanced competition tasks, you will be asked to localize your mbot in a map where you do not know the initial position. This will require initializing your particles in some distribution in open space on the map, and converging on a pose. This is useful in the competition but **does not required any submission in the Checkpoint 3**.

### TODO

{: .required_for_report } 
Explain the methods used for initial localization.

## Checkpoint Submission

Demonstrate your path planner (task 3.1) by showing your robot navigating a maze while avoiding obstacles.
- Submit a video of your robot autonomously navigating in a maze environment.
   - Your video should include the following:
      - The robot driving in the real-world lab maze.
      - Your visualization tool (RViz or Foxglove) displaying the map, planned path, and actual path.