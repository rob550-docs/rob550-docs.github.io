---
layout: default
title: mbot_vision Guide
parent: How-to Guide
grand_parent: Botlab
nav_order: 1
last_modified_at: 2024-11-05 16:37:48 -0500
---

> This guide explains how to use the `mbot_vision` example code.

### Contents
* TOC
{:toc}

## Prerequisite
Run the following commands to pull the latest `mbot_lcm_base` updates:
```
$ cd ~/mbot_ws/mbot_lcm_base
$ git pull
$ ./scripts/install.sh
```

If you have pushed the GitHub mbot_lcm_base repository to your group’s GitLab repository, you can still pull from the GitHub upstream:
1. Run `git remote -v` to check the old GitHub remote’s name, for example, "old-origin".
2. Run `git fetch old-origin` to get the latest updates from GitHub.
3. Run `git merge old-origin/main` to merge the updates from the main branch of "old-origin" to your current branch. For instance, if you are on a branch named "test", the updates will merge into your "test" branch.
4. Run `git push` to push the changes to your GitLab repository.

## Video Demo
<iframe width="560" height="315" src="https://www.youtube.com/embed/cVtnwtbuxPw?si=6Wio0GMkb4QAe3Ma" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Installation
### Installing the Apriltag Library
First, clone the Apriltag library from the official [GitHub repository](https://github.com/AprilRobotics/apriltag):
```bash
$ git clone https://github.com/AprilRobotics/apriltag.git
```

Then, follow the installation [instructions](https://github.com/AprilRobotics/apriltag?tab=readme-ov-file#install) to install the apriltag, run the following commands:
```bash
$ cd apriltag
$ cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF
$ sudo cmake --build build --target install
$ echo 'export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.11/site-packages' >> ~/.bashrc
```

### Installing YOLO
```bash
pip install ultralytics --break-system-packages
pip install --no-cache-dir "ncnn" --break-system-packages
echo 'export PYTHONPATH=$PYTHONPATH:/home/mbot/.local/bin' >> ~/.bashrc
source ~/.bashrc
```

### Cloning `mbot_vision` repository
```bash
$ cd ~/mbot_ws
$ git clone https://gitlab.eecs.umich.edu/ROB550-F24/mbot_vision
```

## Testing the Setup
To test your camera, **open a vscode new terminal**, then execute the command below and then visit `http://your_mbot_ip:5001` in your web browser:
```bash
$ cd ~/mbot_ws/mbot_vision
$ python3 video_streamer.py
```
- This script is for testing the video stream to ensure your camera is operational.

{: .important }
Close this script when you're done with it to free up the camera. You can only ever have one camera script running at a time. This applies to the following scripts as well.

## Camera Calibration
To find the intrinsic matrix of the mbot camera, we need to perform the camera calibration.

1. Find a Checkerboard in the lab

    The size of the checkerboard is determined by the number of intersections, not squares. For instance, a notation of [8, 6] means the board has 8 intersections along one dimension and 6 along the other, corresponding to a grid of 9 by 7 squares. You need this information to use the code.

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
    CHECKERBOARD = (6,8)
    square_size = 25    # in millimeters
    ```
    Then run the following command:
    ```bash
    $ python3 camera_calibration.py
    ```
    - This program uses the images in `/images` to generate the calibration data `cam_calibration_data.npz`.

4. Verify Calibration Results
    - A "Mean reprojection error" below 0.5 indicates successful calibration, and the camera matrix is reliable.
    - If the "Mean reprojection error" is significantly above 0.5, verify the accuracy of your CHECKERBOARD dimensions and square_size settings.

## Object Detection Demo
Run the following command and visit `http://your_mbot_ip:5001`
```bash
$ python3 tag_cone_detection.py
```
- It runs apriltag and cone detection. When a tag or a cone is detected, the pose estimation results will be displayed on the screen.
- The weights we provide is under `utils/cone_detection_model.pt`, you can train your own model for better result.
- You can change the fps and image resolution in `utils/config.py` to save computational resources.

{: .important}
However, once you change the camera_config's `image_width` and `image_height`, you have to re-calibrate the camera by saving new set of images with the new resolution, because the camera’s intrinsic parameters are resolution-dependent.

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

    If the above steps don't work, try adding the "libapriltag.so.3" to PATH like this:
    ```bash
    $ sudo find /usr -name libapriltag.so*
    ```
    ```bash
    $ export LD_LIBRARY_PATH=/usr/local/lib:libapriltag.so.3
    ```

2. Update the Library Cache

    If the library is in a standard location but still not found, you may need to update your system's library cache. Run the following command to refresh it:

    ```bash
    $ sudo ldconfig
    ```
    - After updating, try running `python3 tag_cone_detection.py` again to check if the issue is resolved.

The values displayed on the screen are in **millimeters**. You can use a physical tape measure to verify the accuracy of the detection.

## Detection LCM Message

By running the following commands, you can publish and subsequently listen to the detection messages:
```bash
# Publish detection messages
$ python3 tag_cone_lcm_publisher.py
# Listen to detection messages
$ python3 tag_cone_lcm_subscriber.py
```
## Code Explanation
### Coordinate frame
- World Frame: the center of the AprilTag is used as the origin of the world frame. The x, y, and z axes are aligned with the tag's edges (horizontal, vertical, and outward, respectively).
- Camera Frame: x pointing to the right, y pointing downward, and z pointing forward (along the optical axis of the camera).

The `cv::solvePnP()` returns the rotation and the translation vectors that transform a 3D point expressed in the object coordinate frame to the camera coordinate frame. Namely, the translation and rotation vectors describe the AprilTag’s position and orientation in the camera's coordinate frame.
