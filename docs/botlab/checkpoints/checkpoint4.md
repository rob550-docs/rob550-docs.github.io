---
layout: default
title: Checkpoint 4
nav_order: 5
parent: Checkpoints
grand_parent: Botlab
last_modified_at: 2025-10-23 14:37:48 -0500
---
This checkpoint still under editing
{: .fs-5 .text-red-200 .fw-500}

The MBot has a camera that can be used to perform vision tasks like identifying Apriltags and other obstacles in the environment.

### Contents
* TOC
{:toc}

## Computer Vision
### Task 4.1 Camera Calibration

1. Find a checkerboard in the lab

    The size of the checkerboard is determined by the number of intersections, not squares. For instance, a notation of [8, 6] means the board has 8 intersections along one dimension and 6 along the other, corresponding to a grid of 9 by 7 squares. You need this information.
2. Run the camera node in the **VSCode Terminal**:
    ```bash
    ros2 run camera_ros camera_node --ros-args \
    -p orientation:=180 \
    -p width:=640 -p height:=480 \
    -p format:=BGR888
    ```
3. **On NoMachine**, run the following command, the GUI will pop up.
    ```bash
    ros2 run camera_calibration cameracalibrator \
    --size 6x8 --square 0.025 --ros-args -r image:=/camera/image_raw -p camera:=/camera
    ```
    - `--size` is the num of intersections we introduced at the step 1.
    - `--square` is the size of the block in meter. Here 0.025 means the block is 25 mm.
    - Adjust the values based on your checkerboard.
4. Use this [link](https://docs.nav2.org/tutorials/docs/camera_calibration.html#tutorial-steps) for reference if you don't know how to use the calibrator.
    - Read from: Tutorial Steps -> 5- Start the camera calibration node
5. Once you get the calibration result, move it to the mbot workspace:
    ```bash
    cp /tmp/calibrationdata.tar.gz ~/mbot_ws/src/mbot_vision
    cd ~/mbot_ws/src/mbot_vision
    tar -xvf calibrationdata.tar.gz -C calibration/
    ```
    - This will unzip the calibration results to `/calibration`.
6. Rebuild the packages so the calibration file is accessible anywhere:
    ```bash
    cd ~/mbot_ws
    colcon build
    source install/setup.bash
    ```

### Launch files

To save time on `ros2 run` all the node individually, we also provide launch files.

- To launch camera node only, run the following in the **VSCode Terminal**:
    ```bash
    ros2 launch mbot_vision camera.launch.py
    ```
    - This will publish topics `/camera/image_raw` and `/camera/camera_info`
    - You can view the camera feed in rqt after launching this node.
- To launch the camera with image rectification using the calibration results (camera node + rectification node), run the following in the **VSCode Terminal**:
    ```bash
    ros2 launch mbot_vision camera_rectify.launch.py
    ```
    - You need to do camera calibration first to run the rectification node. It needs the camera info.
    - This will also publish the topic `/image_rect`
    - You can view the camera feed in both raw and rectified in rqt after launching these 2 nodes.
- To launch full vision pipeline (camera + rectification + AprilTag detection), run the following in the **VSCode Terminal**:
    ```bash
    ros2 launch mbot_vision apriltag.launch.py
    ```
    - This will also publish AprilTag detection results on `/detections`


## Checkpoint Submission
No submission is required for this checkpoint.