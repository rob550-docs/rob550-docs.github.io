---
layout: default
title: Import Github Repos to Gitlab
parent: Staff Guide
nav_order: 3
last_modified_at: 2023-10-19 14:20:48 -0500
---

> This is a guide about how do we use the github code base on gitlab with MBot Classic, what are the steps you need to do.

## Checklists
1. Modification needed to use mbot_firmware for mbot classic
    - Check if the robot type has changed to diff drive in `src/mbot.h`: #define MBOT_DRIVE_TYPE DIFFERENTIAL_DRIVE
2. IP registry
    - If IP registry token has expired, it needs to be updates in `mbot_sys_utils/mbot_config.txt`
3. Change/check the MOTION CONTROLLER in `mbot_autonomy/CMakeLists.txt` 