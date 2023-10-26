---
layout: default
title: LCM Guide
parent: Student Guide
nav_order: 5
last_modified_at: 2023-10-19 15:37:48 -0500
---

> The guide introduces you how to leverage LCM for your project.

### Contents
* TOC
{:toc}


### How to add new LCM type and channel

> Assuming you have followed the *MBot System Setup* guide, you should already have cloned both `mbot_lcm_base` and `mbot_firmware` to your Mbot Jetson.

1. All the LCM message types are stored in the `mbot_lcm_base/mbot_msgs/lcmtypes` directory. To add a new LCM type, define your customized LCM message in that location. 
2. Assign a channel name in `mbot_lcm_base/mbot_lcm_serial/include/mbot_lcm_serial/lcm_config.h`. 

    ```markdown
    #define MBOT_EXAMPLE_CHANNEL "MBOT_EXAMPLE"
    ```
    - Here `MBOT_EXAMPLE_CHANNEL` is a constant that stores the string `"MBOT_EXAMPLE"`. Use of these defined constants instead of the raw strings in the rest of the code base helps to avoid errors due to mistyping.

3. Add the channel to the enum in `mbot_lcm_base/mbot_lcm_serial/include/mbot_lcm_serial/lcm_config.h`. 

    ```cpp
    enum message_topics{
        ...
        MBOT_EXAMPLE = 301
    };
    ```

    {: .warning }
    All 200-level numbers are reserved for development purposes only. Therefore, students should **only** use 300-level numbers when creating new channels.
4. Make and install `mbot_lcm_base` by running
    ```bash
    $ cd ~/mbot_ws/mbot_lcm_base
    $ ./scripts/install.sh
    ```
    Now your lcm base is good to go. Next we want to update the firmware base.

5. Add the same channel and number from step 3 to the enum in `mbot_firmware/comms/include/comms/mbot_channels.h`
6. Register new LCM channel by adding it to `register_topics()` in `mbot_firmware/src/mbot.c`
7. Make and upload the firmware by executing the following commands same as introduced in the system setup:
    ```bash
    # Compile the firmware
    $ cd ~/mbot_ws/mbot_firmware/build
    $ cmake ..
    $ make  
    # Upload to the control board
    $ cd ~/mbot_ws/mbot_firmware
    $ sudo ./upload.sh build/src/mbot.uf2
    ```