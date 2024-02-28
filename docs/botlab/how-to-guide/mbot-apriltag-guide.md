---
layout: default
title: mbot_apriltag Guide
parent: How-to Guide
grand_parent: Botlab
nav_order: 1
last_modified_at: 2024-02-28 13:37:48 -0500
---

> This guide explains how to use the `mbot_apriltag` example code.

### Contents
* TOC
{:toc}

## Video Tutorial
<iframe width="560" height="315" src="https://www.youtube.com/embed/lj6BLw9VjKw?si=65Hc5XuWc4SzB3yd" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

## Installation
### Installing the Apriltag Library
First, clone the Apriltag library from the official [GitHub repository](https://github.com/AprilRobotics/apriltag):
```bash
$ git clone https://github.com/AprilRobotics/apriltag.git
```

Then, follow the installation [instructions](https://github.com/AprilRobotics/apriltag?tab=readme-ov-file#install) to execute the commands below:
```bash
$ cd apriltag
$ cmake -B build -DCMAKE_BUILD_TYPE=Release
$ sudo cmake --build build --target install
```

### Cloning `mbot_apriltag` repository
```bash
$ cd ~/mbot_ws
$ git clone https://gitlab.eecs.umich.edu/rob550-f23/mbot_apriltag
```

Your codebase is now ready for use!

## Testing the Setup
To test your camera, execute the command below and then visit `http://your_mbot_ip:5001/video` in your web browser:
```bash
$ python3 video_streamer.py
```
- This script is for testing the video stream to ensure your camera is operational and the installation was successful.

### Troubleshooting
If you encounter the following runtime error: "ImportError: libapriltag.so.3: cannot open shared object file: No such file or directory," follow these steps:

1. Verify the Installation Location

    The library is typically installed in /usr/local/lib or /usr/lib. Check these locations:
    ```bash
    $ ls /usr/local/lib | grep libapriltag
    ```
    ```bash
    $ ls /usr/lib | grep libapriltag
    ```
    - If you see "libapriltag.so.3" in the output, proceed to the next step.

2. Update the Library Cache

    If the library is in a standard location but still not found, you may need to update your system's library cache. Run the following command to refresh it:

    ```bash
    $ sudo ldconfig
    ```
    - After updating, try running video_streamer.py again to check if the issue is resolved.


## Camera Calibration
To find the intrinsic matrix of the mbot camera, we need to perform the camera calibratoin.

1. Find a Checkerboard

    Obtain a checkerboard from the lab. The size of the checkerboard is determined by the number of intersections, not squares. For instance, a notation of [8, 6] means the board has 8 intersections along one dimension and 6 along the other, corresponding to a grid of 9 by 7 squares.

2. Collect Images

    Execute the following command and navigate to `http://your_mbot_ip:5001` to gather calibration images:
    ```bash
    $ python3 save_image.py
    ``` 
    - This will save the images in the `/images` directory for camera calibration.
    - Aim to capture images from various angles and distances for optimal results.

3. Start Calibration

    Adjust the code in `camera_calibration.py` to match your checkerboard's specifications:
    ```python
    # TODO: 
    # 1. Adjust the CHECKERBOARD dimensions according to your checkerboard
    # 2. Set the correct square_size value
    CHECKERBOARD = (7,9)    
    square_size = 20    # in millimeters
    ```
    Then run the following command:
    ```bash
    $ python3 camera_calibration.py 
    ```
    - This uses the images in `/images` to generate the calibration data `cam_calibration_data.npz`, which is automatically utilized by `apriltag_streamer.py`. There's no need for further modifications.
    - Uncomment the print statements in the code if you wish to view the calibration results.

4. Verify Calibration Results
    - A "Mean reprojection error" below 0.5 indicates successful calibration, and the camera matrix is reliable.
    - If the "Mean reprojection error" is significantly above 0.5, verify the accuracy of your CHECKERBOARD dimensions and square_size settings.

## AprilTag Detection
After completing the camera calibration process, we obtain the intrinsic matrix. This matrix is essential for mapping points from the 3D camera coordinate system onto the 2D image plane. Utilizing this information, the OpenCV `solvePnP` function enables us to estimate the pose of a 3D object relative to the camera frame, based on its position in the world frame.

In our code, we designate the center of the AprilTag as the origin of the world frame. The x-axis and y-axis are defined by the tag's horizontal and vertical lines, respectively, while the z-axis extends outward from the surface of the tag.

Run the following command and visit `http://your_mbot_ip:5001/video`
```bash
$ python3 apriltag_streamer.py
```
- It runs apriltag detection. When a tag is detected, the pose estimation results will be displayed on the screen.

You should now have a basic understanding of how AprilTag detection operates and how it can be applied in our projects. The values displayed on the screen are in **millimeters**. You can use a physical tape measure to verify the accuracy of the detection.

If the values appear inaccurate, potential reasons include:
1. The camera calibration might not have been performed accurately.
2. The dimensions of the images used during operation may not match the dimensions of the images used during camera calibration (referenced as save_image.py).
    ```python
    # default settings:
    image_width = 1280
    image_height = 720
    ```

## AprilTag LCM Message

By running the following commands, you can publish and subsequently listen to AprilTag messages using the LCM framework:
```bash
# Publish apriltag lcm message over MBOT_APRILTAG_ARRAY
$ python3 apriltag_lcm_publisher.py
# Listen to MBOT_APRILTAG_ARRAY for apriltag lcm message
$ python3 apriltag_lcm_subscriber.py
```

The lcm message structure can be found under `mbot_lcm_base/mbot_msgs/lcmtypes`. 

Side Note #1: If you are familiar with ROS and are looking for functionality similar to `ros2 topic echo`, LCM does not offer a direct command-line tool for echoing messages. To observe the content of messages, you have to implement a subscriber script.

Side Note #2: To learn more about LCM, we have 2 posts might be helpful: [LCM for Beginner](/docs/staff-guide/lcm-for-beginner) and [Advanced LCM Guide](/docs/botlab/how-to-guide/advanced-LCM-guide).

