---
layout: default
title: Advanced LCM Guide
parent: How-to Guide
grand_parent: Botlab
nav_order: 4
last_modified_at: 2024-02-28 14:37:48 -0500
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
        MBOT_EXAMPLE = 200
    };
    ```

    {: .warning }
    The channel number is limited to a maximum of 255 due to the 8-bit limit. The numbers 0 and 255 might reserved for special purposes. Therefore, 254 is your highest usable value for channel identifiers. 

4. Add the new defined type to `mbot_msgs/CMakeLists.txt`:
```cpp
set(LCM_FILES
  ...  
  lcmtypes/slam_status_t.lcm
  lcmtypes/exploration_status_t.lcm
  ...
  lcmtypes/mbot_example_t.lcm
)
```

5. Make and install `mbot_lcm_base` by running
    ```bash
    $ cd ~/mbot_ws/mbot_lcm_base
    $ ./scripts/install.sh
    ```
    Now your lcm base is good to go. Next we want to update the firmware base.

6. Add the same channel and number from step 3 to the enum in `mbot_firmware/comms/include/comms/mbot_channels.h`
7. Register new LCM channel by adding it to `register_topics()` in `mbot_firmware/src/mbot.c`, remember to define the callback functions for subscriptions.

8. Make and upload the firmware by executing the following commands same as introduced in the system setup:
    ```bash
    # Compile the firmware
    $ cd ~/mbot_ws/mbot_firmware/build
    $ cmake ..
    $ make  
    # Upload to the control board
    $ cd ~/mbot_ws/mbot_firmware
    $ sudo ./upload.sh build/src/mbot.uf2
    ```

9. To publish your newly defined message from mbot_autonomy, you need to use an LCM instance to either publish to or subscribe from channels with the messages or callback functions.

    Check the code in `mbot_autonomy/src/planning/motion_planner_server.cpp` as example
    ```cpp
    //...
    lcm_.subscribe(PATH_REQUEST_CHANNEL, &MotionPlannerServer::handleRequest, this);
    //...
    lcm_.publish(CONTROLLER_PATH_CHANNEL, &path);
    ```
    - `lcm_.subscribe` listens for messages on PATH_REQUEST_CHANNEL. When a message is received, it triggers the callback function `handleRequest`. The `this` pointer refers to the current instance of the class.
        - `The MotionPlannerServer::handleRequest` callback function is where we define the preferred actions to take when data arrives.
    - `lcm_.publish` sends the `path` message on CONTROLLER_PATH_CHANNEL.
