---
layout: default
title: Camera Calibration
parent: How-to Guide
nav_order: 1
last_modified_at: 2023-11-06 18:37:48 -0500
---

> This guide introduces how to execute the camera calibration process using the `mbot_apriltag` code example provided.

{: .highlight}
For the camera calibration process, you need to either connect the Jetson to an external monitor or access it using NoMachine.

1. Clone the code base
```bash
$ cd mbot_ws
$ git clone https://gitlab.eecs.umich.edu/rob550-f23/mbot_apriltag
```

2. Install
```bash
$ cd mbot_apriltag/
$ sudo ./install.sh
```

3. Find a checkerboard

    You will need a checkerboard of a size specified in the `calibration_config.yaml` file. The size of the checkerboard is defined by the number of points. For example, [8, 6] indicates that the board has 8 points by 6 points, which corresponds to 9 blocks by 7 blocks.

4. Start calibration
```bash
$ sudo python3 calibrate_camera.py
```
- When you press `SPACE`, one frame is captured if there is a detection, and detected corners will be displayed in the window. To ensure the checkerboard is detected, you need to make sure all the points are in view.
- After capturing more than 10 frames, the window will prompt you to press `ENTER` to start calibration. You can continue to capture more frames for a better result.
- For the best results, try to include a variety of angles and distances.
- The calibration result will be saved to the file `camera_params.yaml`.
    <iframe width="560" height="315" src="https://www.youtube.com/embed/Ql5zrzOgYYw?si=TDCcuK3xRbw09CdP" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

5. Check the calibration result
```bash
$ sudo python3 view_apriltag.py
```
In the video below, you can tell that the edge of the frame is off from the window because we used `cv2.undistort` to undistort the image with the calibration result.
    <iframe width="560" height="315" src="https://www.youtube.com/embed/INiWHoyHXDs?si=FkSA8FAHOff3Fyrp" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>