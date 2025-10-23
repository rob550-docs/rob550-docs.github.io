---
layout: default
title: Checkpoint 2
nav_order: 3
parent: Checkpoints
grand_parent: Botlab
last_modified_at: 2025-10-23 14:16:00 -0500
---

This checkpoint still under editing
{: .fs-5 .text-red-200 .fw-500}

During the SLAM part of the lab, you will build an increasingly sophisticated algorithm for mapping and localizing in the environment. You will begin by constructing an occupancy grid using known poses. Following that, you’ll implement Monte Carlo Localization in a known map. Finally, you will put each of these pieces together to create a full SLAM system.


### Contents
* TOC
{:toc}

## Preparation

1. Pull from the [mbot_ros_labs](https://gitlab.eecs.umich.edu/rob550-f25/mbot_labs_ws) upstream to get the template code for checkpoint 2. 
2. Read the guide on how to use Foxglove: [link](). Foxglove is a web-based visualization tool that provides all the functionalities of RViz. We offer it as an alternative if RViz feels slow or cumbersome when running through NoMachine.

## Task 2.1 Mapping

An occupancy grid maps the space surrounding the robot by assigning probabilities to each cell, indicating whether they are occupied or free. In our case, we'll use a grid where cells are at most 10 cm in size, and their log-odds values range from -127 to 127, represented as integers. To accurately trace the rays across your occupancy grid, consider implementing Bresenham’s line algorithm, as discussed in our class lecture.

### TODO

{: .required_for_report } 
Include the map screenshot.


## Task 2.2 Localization
As discussed in the lecture, Monte Carlo Localization (MCL) is a particle-filter-based localization algorithm. Implementation of MCL requires three key components: 
- An action model to predict the robot’s pose
- A sensor model to calculate the likelihood of a pose given a sensor measurement
- Various functions for particle filtering, including drawing samples, normalizing particle weights, and finding the weighted mean pose of the particles. 

In these tasks, you’ll run the slam program in localization-only mode using a saved map. Use the ground-truth maps provided with the sensor logs. You can run slam using: 
```bash
cd ~/mbot_ws/mbot_autonomy/build
./mbot_slam --localization-only --map <map_file.map>
```

To test the action model only, you can run in action-only mode:
```bash
./mbot_slam --action-only
```

### Task 2.2.1 Action Model & Particle Filter
Implement an action (or motion) model using some combination of sensors. This could be odometry with wheel encoders alone, or it could be some other estimate of action incorporating the IMU or using computer vision. The skeleton of the action model can be found in `mbot_autonomy/src/slam/action_model.cpp|hpp`.

Refer to Chapter 5 of Probabilistic Robotics for a discussion of common action models. You can base your implementation on the pseudo-code in this chapter. There are two action models that are discussed in detail, the Velocity Model (Sec. 5.3) and the Odometry Model (Sec. 5.4).

You will also need to implement several functions in `mbot_autonomy/src/slam/particle_filter.cpp|hpp` to have your action model affect the particles.

If you test in action-only mode you will see the particles spread out over time. That is expected. The action model adds randomness to your position, so your estimated position will get worse and worse if you don't use the sensor model to filter your position.

{: .required_for_report } 
Describe the action model you utilized, including the equations employed. Additionally, provide a table listing the values of any uncertainty parameters, and explain how you chose these values.

### Task 2.2.2 Sensor Model & Particle Filter
Implement a sensor model that calculates the likelihood of a pose given a laser scan and an occupancy grid map. The skeleton of the sensor model can be found in `mbot_autonomy/src/slam/sensor_model.cpp|hpp`.  

Refer to Chapter 6 of Probabilistic Robotics for a discussion of common sensor models. You can base your implementation on the pseudo-code in this chapter.  

Implement the remainder of the particle filter functions in `mbot_autonomy/src/slam/particle_filter.cpp|hpp`.

You can use the helpful resampling functions `lowVarianceSample` and `importanceSample` to resample your posterior distribution.

Refer to Chapter 4 of Probabilistic Robotics for information implementing the particle filter.

Hint: In case of slow performance of your sensor model, consider increasing the ray stride in the MovingLaserScan call.

{: .required_for_report } 
1) Report in a table the time it takes to update the particle filter for 100, 500 and 1000 particles. Estimate the maximum number of particles your filter can support running at 10Hz.
<br> 2) Using your particle filter on `mbot_example_logs/botlab/drive_square.log`, plot 300 particles at regular intervals along the path taken.

## Task 2.3 Simultaneous Localization and Mapping (SLAM)
You have now implemented mapping using known poses and localization using a known map. You can now run the slam program in SLAM mode, which uses the following simple SLAM algorithm:
- Use the first laser scan received to construct an initial map.
- For subsequent laser scans:
    - Localize using the map from the previous iteration.
    - Update the map using the pose estimate from your localization.

NOTE: The above logic is already implemented for you. Your goal for this task is to make sure your localization and mapping are accurate enough to provide a stable estimate of the robot pose and map. You will need good SLAM performance to succeed at the robot navigation task.

{: .required_for_report } 
1) Create a block diagram of how the SLAM system components interact
<br> 2) Compare the estimated poses from your SLAM system against the ground-truth poses in `drive_maze_full_rays.log`. Use this to estimate the accuracy of your system and include statistics such as RMS error etc.


## Checkpoint Submission
<br>
<a class="image-link" href="/assets/images/botlab/checkpoints/checkpoint1-maze.png">
<img src="/assets/images/botlab/checkpoints/checkpoint1-maze.png" alt=" " style="max-width:600px;"/>
</a>

Demonstrate your SLAM system by mapping the maze used in Checkpoint 1. You may either manually drive the robot using teleop or use the motion controller from Checkpoint 1.
1. Submit a screenshot of the generated map.
2. Submit the map file itself.
3. Submit a short description of your SLAM system, including how it works and any key observations.