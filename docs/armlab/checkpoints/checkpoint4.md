---
layout: default
title: Checkpoint 4
nav_order: 4
parent: Checkpoints
grand_parent: Armlab
last_modified_at: 2024-02-26 15:00:00 -0500
---

### Contents
* TOC
{:toc}

## Intro
Now, you need to put all the pieces together.  

You will need to write a simple planner to control the motion of the arm and implement states in the state machine to control the arm for picking and placing blocks. Given a set of target locations from the block detector, your arm will need to move into a position from which it may grab the target block, then move to the drop-off location. 

Keep in mind that the actual end effector position may be different from the target locations given by the block detector.

This will be similar to the teach-and-repeat planner, except you will need to automatically generate the waypoints based on the output of the block detector, kinematics, and your strategy for the final competition.

## Task 4.1 Final Tuning
Implement logic for the final competitions into your state machine.

{: .required_for_report }
1) Evaluate the performance of your system during the competition. Include performance and discussion on the detection and manipulation of the arch and semi-circle blocks. <br>
2) Discuss what improvements could be made to improve performance

## Checkpoint Submission
Nothing to submit! Get ready for competition and enjoy : D