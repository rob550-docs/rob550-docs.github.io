---
layout: default
title: MBot Codebase Use
parent: Staff Guide
nav_order: 4
last_modified_at: 2023-10-19 14:20:48 -0500
---

> This guide explains how to use the GitHub codebase for MBot Classic in ROB550.

{: .note}
The reason for this post is that many classes using MBot are pulling and contributing code from the MBot organization on GitHub. However, the code pulled directly from these repositories may not be directly applicable to MBot Classic. Specific modifications are needed to run the code without errors.


## Checklists
1. Modification needed to use mbot_firmware for mbot classic
    - Check if the robot type has changed to diff drive in `src/mbot.h`: #define MBOT_DRIVE_TYPE DIFFERENTIAL_DRIVE
2. IP registry
    - If IP registry token has expired, it needs to be updates in `mbot_sys_utils/mbot_config.txt`
3. Change/check the MOTION CONTROLLER in `mbot_autonomy/CMakeLists.txt` 
4. ...

This list is not exhaustive. Please add new items if needed.