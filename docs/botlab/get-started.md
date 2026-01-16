---
layout: default
title: Get Started with ROS MBot
parent: Botlab
nav_order: 0
last_modified_at: 2026-01-16 12:20:00 -0500
---

### Hardware

First, please follow the [hardware setup guide](/docs/botlab/mbot-hardware-setup-Pi5). You **can skip** the "Extra Wiring Step: LiDAR Power Update" part if you don't have the LiDAR power cable.
- If you do the "Extra Wiring Step: LiDAR Power Update" part, LiDAR pub rate is default to 10 HZ.
- If you skip the "Extra Wiring Step: LiDAR Power Update" part, LiDAR pub rate is default to 7 HZ.


### Software

Note for ROS2 MBot Classic **non-UM users** and **UM instructors outside ROB 550**:
- All Botlab documentation is compatible only with ROS2 MBot Classic systems running **at least** the following versions:


| Repository | Version | Visibility | Note |
| ---        | ---     | ---        | ---  |
|[mbot_firmware_ros](https://github.com/mbot-project/mbot_firmware_ros)|v1.1.5| Public| Firmware code |
|[mbot_ros2_ws](https://github.com/mbot-project/mbot_ros2_ws)|v1.0.3| Public| ROS ecosystem|
|mbot_ros_labs|W26| Private | Lab Assignments |

How to check your version:
- Check the file VERSION in the repository root.
- If it is missing, your copy is outdated, pull the latest release.

To follow MBot software updates, go to the repository and select Watch → Custom → Releases on GitHub. You will receive an email whenever a new release is published. **We recommend that instructors follow the repositories, as new releases may be published during the semester.**

<a class="image-link" href="/assets/images/botlab/github-watch.png">
<img src="/assets/images/botlab/github-watch.png" alt="" style="max-width:500px;"/>
</a>

---
