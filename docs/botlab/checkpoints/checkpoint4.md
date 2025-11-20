---
layout: default
title: Checkpoint 4
nav_order: 5
parent: Checkpoints
grand_parent: Botlab
last_modified_at: 2025-11-20 15:09:48 -0500
---

The MBot is equipped with a camera that can be used for vision-based tasks, such as identifying AprilTags and detecting obstacles in the environment.

### Contents
* TOC
{:toc}

## Computer Vision
### Task 4.1 Camera Calibration

1. Find a checkerboard in the lab.

    **The checkerboard size is defined by the number of intersections, not the number of squares.**

    For example, size 8x6 means the board has 8 intersections along one dimension and 6 along the other, corresponding to a grid of 9×7 squares. You’ll need this information for calibration.
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
    - `--size` specifies the number of intersections (as noted in Step 1).
    - `--square` defines the size of each checkerboard square in meters.
    - Adjust the values based on your checkerboard. 0.025 means each square is 25 mm.
4. How to use the calibrator GUI? Refer to the [ROS2 Camera Calibration Tutorial](https://docs.nav2.org/tutorials/docs/camera_calibration.html#tutorial-steps)
    - Start reading from: Tutorial Steps → Step 5: Start the camera calibration node.
5. Once calibration is complete, move the results to your MBot workspace by running the following commands:
    ```bash
    cp /tmp/calibrationdata.tar.gz ~/mbot_ws/src/mbot_vision
    cd ~/mbot_ws/src/mbot_vision
    tar -xvf calibrationdata.tar.gz -C calibration/
    ```
    - This will unzip the calibration results to `~/mbot_ws/src/mbot_vision/calibration`.
6. Rebuild the packages so the calibration file is accessible system-wide:
    ```bash
    cd ~/mbot_ws
    colcon build
    source install/setup.bash
    ```

### Launch files

To save time on `ros2 run` all the node individually, you can use the provided launch files.

- To launch camera node only, run the following in the **VSCode Terminal**:
    ```bash
    ros2 launch mbot_vision camera.launch.py
    ```
    - This will publish topics `/camera/image_raw` and `/camera/camera_info`
    - You can view the camera feed in rqt on NoMachine or Foxglove after launching this node.
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
    - This will publish AprilTag detection results on `/detections`.
    - It will also publish the TF from `/camera` to each Apriltag, so you do not need to decode `/detections` or convert tag coordinates from the 2D image plane into the 3D world frame yourself.

You can visualize the camera view in rqt on NoMachine desktop:
```bash
ros2 run rqt_image_view rqt_image_view
```

Or on foxglove studio by running the bridge:
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### Video Demo
<iframe width="560" height="315" src="https://www.youtube.com/embed/6qcvDb_q0TQ?si=3X5i8moHI6Qk0ZgI" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Checkpoint Submission
No submission is required for this checkpoint.

However, **ensure your camera can successfully detect AprilTags, you’ll need this for the competition.**