---
layout: default
title: Armlab Camera Stuff
nav_order: 4
grand_parent: Armlab
parent: How-to Guide
last_modified_at: 2024-09-09 16:37:48 -0500
---

{: .note}
This guide covers the use of the RealSense L515 camera in Armlab. It’s for students who want to troubleshoot or get the most out of the camera.

### Contents
* TOC
{:toc}

## RealSense Viewer

To launch the viewer, run:
```bash
realsense-viewer
```
This is the official GUI for viewing the camera stream. Use it if the camera isn’t streaming properly on the control station. If the stream looks good in the RealSense Viewer, the camera is functioning properly.

### User Interface
<br>
<a class="image-link" href="/assets/images/armlab/how-to-guide/camera1.png">
    <img src="/assets/images/armlab/how-to-guide/camera1.png" alt="" style="max-width:600px;"/>
</a>

In the image above:
- **Red box**: Camera model and USB version. Ensure USB 3.2, not USB 2.0, for the launch file to work correctly.
- **Green box**: RGB and Depth cameras. Both are off in the image, so the view is blank.
- **Yellow box**: 2D/3D selection. Choose 2D to check the 2D stream.

<a class="image-link" href="/assets/images/armlab/how-to-guide/camera2.png">
    <img src="/assets/images/armlab/how-to-guide/camera2.png" alt="" style="max-width:600px;"/>
</a>

- **L500 Depth Sensor**: It’s off here, allowing you to see available resolution options.
- **RGB Camera**: It is on and streaming the checkerboard. You can’t change resolution when the camera is on.

## Armlab Default Settings

If you're troubleshooting camera issues or launch file errors, follow these steps:
1. Open `realsense-viewer` and check that the camera is connected via USB 3.0, not USB 2.0. If only USB 2.0 is available, inform your instructor.
2. Next, if you have USB 3.0 cable, ensure the launch file `install_scripts/config/rs_l515_launch.py` has the correct resolution settings. In the file looking for these 2 lines:
    ```python
    {'name': 'depth_module.profile',         'default': '1024,768,30', 'description': 'depth module profile'},                           
    ...
    {'name': 'rgb_camera.profile',           'default': '1280,720,30', 'description': 'color image width'},
    ```
    - The resolutions above are default. `1024,768` for depth module, `1280,720` for rgb camera. Mismatched resolutions will cause errors in `camera.py`.
3. To apply the changes, copy `rs_l515_launch.py` to the system path:
    ```bash
    cd install_scripts
    ./install_LaunchFiles.sh
    ```
4. Verify that the camera initialization in `src/camera.py` matches with the launch file above:
    ```python
    def __init__(self):
        """!
        @brief      Construcfalsets a new instance.
        """
        self.VideoFrame = np.zeros((720,1280, 3)).astype(np.uint8)
        self.GridFrame = np.zeros((720,1280, 3)).astype(np.uint8)
        self.TagImageFrame = np.zeros((720,1280, 3)).astype(np.uint8)
        self.DepthFrameRaw = np.zeros((720,1280)).astype(np.uint16)
        """ Extra arrays for colormaping the depth image"""
        self.DepthFrameHSV = np.zeros((720,1280, 3)).astype(np.uint8)
        self.DepthFrameRGB = np.zeros((720,1280, 3)).astype(np.uint8)
        ...
    ```
5. Then run the `launch_armlab.sh` launch file. If problems persist, capture screenshots of error messages and send them to course Discord for assistance.