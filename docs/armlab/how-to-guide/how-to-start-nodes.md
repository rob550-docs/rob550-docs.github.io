---
layout: default
title: How to start ROS nodes
nav_order: 2
grand_parent: Armlab
parent: How-to Guide
last_modified_at: 2023-12-13 18:37:48 -0500
---

> Here are the commands you'll need to start the ROS nodes individually.

### To launch individual node
- Start Realsense2 camera node
    ```bash
    $ ros2 launch realsense2_camera rs_l515_launch.py
    ```
- Start AprilTag Dectection node 
    ```bash
    $ ros2 run apriltag_ros apriltag_node --ros-args \
        -r image_rect:=/camera/color/image_raw \
        -r camera_info:=/camera/color/camera_info \
        --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_Standard41h12.yaml
    ```
- Start the arm node
    ```bash
    $ ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=rx200
    ```
    - This command will launch rviz with the virtual robot model, the model would show exactly how the arm moving.
    - More can be learned from the official [ROS 2 Quickstart Guide](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/quickstart.html)
- Start the camera calibration node
    ```bash
    $ cd ~/image_pipeline
    $ source install/setup.bash
    # the size of the checkerboard and the dimensions of its squares may vary
    $ ros2 run camera_calibration cameracalibrator --size 6x8 --square 0.025 \
        --no-service-check --ros-args \
        -r image:=/camera/color/image_raw  \
        -p camera:=/camera
    ```

{: .note}
Before starting any node that requires the camera, make sure to start the Realsense2 node first.