---
layout: default
title: Checkpoint 1.5
nav_order: 3
parent: Checkpoints
grand_parent: Botlab
last_modified_at: 2024-11-05 15:37:48 -0500
---
This checkpoint still under editing
{: .fs-5 .text-red-200 .fw-500}

The MBot has a camera that can be used to perform vision tasks like identifying Apriltags and other obstacles in the environment.

### Contents
* TOC
{:toc}

## Computer Vision
### Task 1.5 Camera Calibration

Find the intrinsic and extrinsic matrix (robot frame to camera frame) for your MBot.

Hints:
- For the intrinsic matrix, use the camera calibration tools available in the [mbot_vision](https://gitlab.eecs.umich.edu/ROB550-F24/mbot_vision) repository.
For detailed instructions on using `mbot_vision`, please refer to [mbot_vision guide](/docs/botlab/how-to-guide/mbot-vision-guide).
- To determine the extrinsic matrix (robot frame to camera frame), utilize the CAD model to determine the offsets.

### Task 1.6 Apriltag Detection

Write a python program to estimate the distance and heading to an Apriltag and maneuver up to the tag.

Hint:
- For accessing the AprilTag detection LCM message, refer to the [mbot_vision guide](/docs/botlab/how-to-guide/mbot-vision-guide).

Compare the camera detection of the apriltag with your odometry by driving backwards and while keeping the Apriltag in sight.


## Checkpoint Submission
No submission is required for this checkpoint.