---
layout: default
title: Checkpoint 2
nav_order: 2
parent: Checkpoints
grand_parent: Armlab
last_modified_at: 2023-12-14 14:37:48 -0500
---

**DUE: 2/8/24**

### Contents
* TOC
{:toc}


## Apriltag Calibration
In the previous checkpoint, we estimated the extrinsic parameters of the camera using physical measurements. In this checkpoint, we will use Apriltags on the board and their known locations instead to estimate the extrinsic parameters.

### Intro
Apriltags are a commonly used fiducial system in robotics and computer vision. For information about Apriltags see [here](https://april.eecs.umich.edu/software/apriltag). 

Each tag is a member of a family and has a specific ID. In order to have the tags detected in an image and the tag location published as a ROS message, You will need to specify the IDs of the tags you wish to detect in the configuration file.

To specify the configuration, you will need to modify the `tags_Standard41h12.yaml` file in `/install_scripts/config` and copy it to `/opt/ros/humble/share/apriltag_ros/cfg/`, which is what we did during the setup guide, by running `./install_LaunchFiles.sh`. This is a global config file so it will affect the settings of the current active user account. The tag's family and ID are displayed on the tag sticker. You can refer to the sticker to check these details, especially when replacing the tag or using additional ones.

Once the AprilTag detection node is initiated, it detects all tags within the camera's field of view and publishes ROS messages with details of the detection, including the tag's ID and location. Next, let's take a look at actual commands and their outputs.

- To determine the message type of the AprilTag detection topic `/detections`, use the following command:
    ```bash
    $ ros2 topic type /detections 
    ```
    - The message type is a data structure that describes the content of the message.
    - The output will indicate that the topic type is `apriltag_msgs/msg/AprilTagDetectionArray`. This is a specific data structure utilized for describing the tag detections and is defined by the package author.
- To check the details of this message type:
    ```bash
    $ ros2 interface show apriltag_msgs/msg/AprilTagDetectionArray
    ```
    - The output would be something like this:
    ```bash
    std_msgs/Header header
            builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
            string frame_id
    AprilTagDetection[] detections
            string family
            int32 id
            int32 hamming
            float32 goodness
            float32 decision_margin
            Point centre                    #
                float64 x
                float64 y
            Point[4] corners                #
                float64 x
                float64 y
            float64[9] homography           #
    ```
- To check the actual value of the message:
    ```bash
    $ ros2 topic echo /detections
    ```
- To learn more about ros2 humble topic commands, check the [official docs](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html).


### Task 2.1 Validate the Apriltag Detection

In the control station GUI, you'll find several views: Video, Depth, Tags, and User2. Your specific task involves the `drawTagsInRGBImage()` function in `src/camera.py`. This function requires you to use OpenCV to draw detections of AprilTags.

Key elements to include in your drawing are:
- The tag's ID.
- A highlight on the tag's center.
- The tag's edges.

An example of the expected result is provided for reference:
<a class="image-link" href="/assets/images/armlab/checkpoints/checkpoint2-validate-apriltag.png"><img src="/assets/images/armlab/checkpoints/checkpoint2-validate-apriltag.png" alt="" style="max-width:600px;"/>
</a>

While your result doesn't have to be identical, the goal is to verify the accuracy of the detection. A good indication of accurate detection is if your drawing aligns well with the real-world objects. This alignment ensures the reliability of the AprilTag detection output.


### Task 2.2 Camera Calibration Using GUI
Add a workspace calibration procedure. This procedure should be initiated by one of the user buttons in the GUI, integrated into the state machine as a `calibration` state. 

The procedure will display status messages at the bottom of the GUI window, which will guide users through the calibration steps. Additionally, pressing this button will not only have the calibration results, but also apply a projective transform to adjust the camera's perspective.

As you may have noticed, the camera is not facing directly down to the board but is at a position and orientation offset. In order to work with our workspace as if you were looking at it from above (which will be much more helpful for the future tasks), you’ll need to apply a projective transformation, to make camera view to appear as though it's viewing from directly above the board, rather than from an angle.

Hints:
- A manual calibration might go as follows (this is the “semi-automatic” calibration):
    1. Use mouse clicks to get pixel locations of known locations in the workspace. Repeat with the depth frame and use an affine transformation to register the two together if necessary.  
    2. Using the intrinsic matrix you find for the RGB camera along with any depth calibration you determine, create a function that takes pixel coordinates from the image and returns world coordinates.
- Alternatively, you may implement an auto-calibration routine using Apriltags visible in the workspace, or possibly by detecting the grid in the image. This is the “automatic” calibration.

After the calibration is performed, update the display under the video in the GUI to show the world coordinates of the mouse location as you hover over the video. Measurements only need to be valid for points on or above the board.

Record for report:
- Report your extrinsic matrix and depth calibration function. 
- Report the equations used to turn [u,v,d] coordinates in the image frames to [x,y,z] coordinates in the world frame.
- Describe how you verified the calibration was correct, what additional steps you took to correct it, and provide evidence for the accuracy of your calibration.

Questions to consider:
- How do the extrinsic matrices compare with the hand-measured ones? 
- What are the sources of error, and how large are they? 
- Is it sufficient to rely on this nominal matrix for finding points in the world?


### Task 2.3 Validate the Calibration
After obtaining the calibration result, we need to validate the result.

Instructions:
- Project a grid of points onto the image. These points should correspond to the corners of the grid on your workspace. 
    - In `camera.py`, you will find the variable `self.grid_points` defining the world coordinates of your workspace's grid corners. Use these for your projection. You should employ the `projectGridInRGBImage()` function to project these grid points onto the image.
- After projecting the grid points, to compare them to the actual corners of the workspace in the image, select the “User 2” radio box in the GUI, which activates the grid view. There's no need to modify `control_station.py` for this functionality as long as you implement the `projectGridInRGBImage()` function correctly. 

Finally, apply a perspective transform. This will change your image's perspective to resemble a bird's-eye view, providing a different angle of the workspace.

Record for report:
- Quantitatively or qualitatively, assess your calibration routine using the grid point projection. 
- Which points did you use for your perspective transformation? Report the homography matrix.

Questions to consider:
- What does each of the entries of your homography matrix represent?

## Forward Kinematics
For this part, you will work on implementing Forward Kinematics to determine the location of the end effector in the global frame.

### Intro
The objective of forward kinematics (FK) is to determine the position of the joints in the global frame, given the position of each of the joints in the robot frame. You can approach the implementation of the FK using either the Denavit-Hartenberg (DH) table or the Product of Exponentials (PoX). Please refer to the lecture slides for further details regarding these methods.

Regardless of the method used, you must use the geometry of the RX200 arm to utilize either of the FK methods. You can find the technical drawings of the RX200 arm for this purpose [here](https://www.trossenrobotics.com/reactorx-200-robot-arm.aspx#drawings).

To implement the forward kinematics, you will work with the `kinematics.py` script, which has helper functions with docstrings for either the DH table or the PoX approach. Either of these methods must take in the 5 joint angles of the RX200 arm and output the wrist location in the global coordinates.

One function (FK_DH) would use a DH table to determine the end effector position. The other (FK_PoX) would take screw vectors and an M-matrix and calculate the end effector position using the product of exponentials formulation. You will likely have to create helper functions to perform the FK calculations.

### Task 2.4 Forward Kinematics
Refer to the technical drawing of the RX200 arm on the [manufacturer's website](https://www.trossenrobotics.com/reactorx-200-robot-arm.aspx#drawings). You must implement one of two forward kinematics functions that take five joint angles and return the wrist location in workspace (global frame) coordinates.  

The end effector position is defined by $$(x, y, z, \varphi, \theta, \psi)$$ where the orientation uses the common ZYZ Euler angle convention.  You should display the coordinates of the center of the gripper (End Effector) in your GUI using the text labels provided in the upper left. 

Record for report:
- Include a DH table or screw vectors and M matrix for the RX200 arm in your report.
- Include a schematic of the arm with DH frames or screw vectors indicated
- Describe how you verified that your arm tooltip achieved expected workspace coordinates for your test cases, estimate the error, and justify the accuracy of your measurements.


## Checkpoint Submission
### Part 1
- Rotate the camera, then demonstrate the calibration procedure in a video
- With the camera rotated, record the reported position (x,y,z) of the center of the top of a stack of large blocks placed at the following locations for a stack size of [0,1,2,4,6] blocks (total of 4x5=20 measurements):
    - (0,175), (-300, -75), (300, -75), (300, 325)

### Part 2
Record a video showing your Forward Kinematics results and arm positions while manually driving the end effector the following positions:
- (0, 175, 100)
- (-300, -75, 25)
- (300, -75, 50)
- (300, 325, 10)
