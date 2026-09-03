---
layout: default
title: Hardware
nav_order: 4
parent: Armlab
last_modified_at: 2026-09-02 12:00:00 -0400
---

> What is at your station: the arm, the gripper, the camera, and how it is all powered and connected.

<!--
TEMPLATE NOTES
- Put images in /assets/images/armlab/hardware/
- Image snippet:
    <a class="image-link" href="/assets/images/armlab/hardware/FILE.png">
    <img src="/assets/images/armlab/hardware/FILE.png" alt="" style="max-width:400px;"/>
    </a>
- Available callouts: .note .important .warning .highlight .new .sanity_check .required_for_report
-->

### Contents
* TOC
{:toc}

## Station Overview
TODO: photo of a full station and a short list of everything on it (arm, board, 24 V supply, camera frame, laptop).

## UFactory Lite 6 Arm
The arm is a **UFACTORY Lite 6**, a 6-DOF collaborative arm with the control electronics built into its base — there is no separate controller cabinet. It is commanded over Ethernet from the station laptop using the xArm Python SDK.

| Specification | Value |
| ------------- | ----- |
| Degrees of freedom | 6 |
| Reach | 440 mm |
| **Payload** | **600 g** |
| Repeatability | ±0.5 mm |
| Weight | 7.2 kg |
| Max TCP speed | 500 mm/s |
| Max joint speed | 180 °/s |
| Power | 24 V DC |
| Communication | Ethernet |
| Tool flange | ISO 9409-1-50 |

Joint travel limits are **not symmetric**:

| Joint | Range |
| ----- | ----- |
| J1 | ±360° |
| J2 | ±150° |
| J3 | −3.5° to 300° |
| J4 | ±360° |
| J5 | **±124°** |
| J6 | ±360° |

{: .important}
J5 (±124°) and J3 (−3.5° to 300°) are the limits you will actually run into. An inverse-kinematics solution can be mathematically correct and still unreachable because it asks one of these joints to go somewhere it cannot. See the [Lite 6 Arm & SDK Guide](/docs/armlab/how-to-guide/lite6arm-sdk-guide) for how this shows up in the software.

{: .warning}
The 600 g payload is the entire budget and **the gripper counts against it**, leaving very little for the object you are picking up.

The arm is otherwise unmodified for this course — the only change is the custom gripper on the tool flange.

{: .important}
The **emergency stop is on the table next to the arm.** Find it before you power the arm on. Safety rules for the lab will be covered by the staff in person.

## Gripper / End Effector
The gripper is **custom-built for this lab**, mounted on the Lite 6's ISO 9409-1-50 tool flange. It is commanded with the standard Lite 6 gripper calls — `open_lite6_gripper()`, `close_lite6_gripper()` and `stop_lite6_gripper()`; see the [Lite 6 Arm & SDK Guide](/docs/armlab/how-to-guide/lite6arm-sdk-guide#the-gripper).

It is a binary open/closed device: no width control, no force control, and no feedback about whether a grasp succeeded.

TODO: mechanism description and photo, maximum block size it can hold, gripper mass (it counts against the 600 g payload), and the tool center point offset from the flange.

## Power and Network
The Lite 6 has no separate controller cabinet — the control electronics are inside the arm's base. Power comes from a **24 V power supply mounted underneath the board** the arm is bolted to.

The arm talks to the station laptop over Ethernet, through a **USB-to-Ethernet adapter**. That adapter is what gives the laptop a second Ethernet port.

| Item | How it connects |
| ---- | --------------- |
| Arm power | 24 V supply mounted under the board |
| Arm data | Ethernet → **USB-to-Ethernet adapter** on the laptop |
| Internet | The laptop's **built-in** Ethernet port |
| Camera | USB 3.x to the laptop |

{: .warning}
**Do not plug the arm into the laptop's built-in Ethernet port.** That port is reserved for internet access and sits on a different IP address and subnet. The arm must go through the USB-to-Ethernet adapter, or it will not be reachable.

Each arm has its own IP address, printed on a label on the back of the arm. You set it in the code during [setup](/docs/armlab/setup-guide#4-set-your-stations-arm-ip).

## RealSense Camera
Perception uses an **Intel RealSense LiDAR Camera L515**. Unlike the stereo RealSense models, the L515 measures depth with a solid-state MEMS LiDAR, which gives it a clean, low-noise depth image at close range — well suited to looking down at blocks on a board.

| Specification | Value |
| ------------- | ----- |
| Depth technology | Solid-state MEMS LiDAR |
| Operating range | 0.25 – 9 m |
| Depth field of view | 70° × 55° |
| Depth stream **used in this lab** | 1024 × 768 @ 30 fps |
| Color stream **used in this lab** | 1280 × 720 @ 30 fps |
| Depth accuracy | ~5 mm at 1 m |
| Connection | USB 3.x (Type-C) |
| Size | 61 mm diameter × 26 mm |

The depth and color streams come from two physically separate sensors, so `camera.py` aligns depth to color before anything else. After alignment a pixel `(u, v)` means the same point in both images.

{: .warning}
**USB 3.x is required.** On a USB 2 port the full-resolution profiles are unavailable and the camera will not stream at the settings the control station asks for. Check the connection type with `rs-enumerate-devices -s`.

{: .note}
The L515 is an **indoor** sensor — its LiDAR competes with ambient infrared, so strong sunlight degrades the depth image. It is also discontinued by Intel, which is why the lab pins librealsense to 2.54.2, the last version that supports it.

The camera runs in the **Short Range** visual preset. Without it the firmware discards depth closer than about 411 mm, which would blank out the near part of the workspace.

See the [Camera Guide](/docs/armlab/how-to-guide/camera-guide) for the RealSense Viewer and calibration.

The camera is mounted on an **aluminum frame above the arm**, looking down at the board. Its aim is adjustable: loosen the screw with a hex key, rotate the camera to where you want it, then tighten the screw again.

{: .important}
Aiming the camera **invalidates your extrinsic calibration**. The calibration solves for where the camera sits relative to the robot, so any time you loosen that screw you have to run Calibrate again before world coordinates mean anything.

TODO: photo of the frame and the adjustment screw, and the approximate mounting height above the board.

## Board and Workspace
The arm is mounted on a board carrying AprilTags at fixed positions.

{: .important}
**Measure the board yourself.** You need the grid spacing, the position of the robot frame origin, the direction of its axes, and the location of each tag relative to that origin. Those measured tag positions are exactly what goes into `TAG_WORLD_POINTS` in `camera.py` — your extrinsic calibration can be no better than these measurements, so take them carefully and record them in your report.

## Reference Links
- [Lite 6 product page and specifications](https://www.ufactory.us/product/lite-6)
- [Lite 6 user manual (PDF)](https://www.ufactory.cc/wp-content/uploads/2023/05/Lite6-User-Manual-V2.0.0.pdf)
- [Intel RealSense L515 product page](https://www.intelrealsense.com/lidar-camera-l515/)
- [L515 datasheet (PDF)](https://www.mouser.com/datasheet/2/612/Intel_RealSense_LiDAR_L515_Datasheet_Rev002-1713847.pdf)
