---
layout: default
title: MBot LCM User Guide
parent: Staff Guide
nav_order: 4
last_modified_at: 2024-10-09 14:20:48 -0500
---

> This guide is about how to use LCM in MBot ecosystem.

### Contents
* TOC
{:toc}

## What is LCM?
[LCM](https://lcm-proj.github.io/lcm/) stands for Lightweight Communications and Marshalling. It's about sending data between programs or computers (Communications) and getting data ready for sending or storing (Marshalling).
- If you know ROS, LCM works similarly with the same publish-subscribe model. LCM types are like ROS messages, and LCM channels are like ROS topics.
- If ROS is new to you, think of LCM as a way to send and receive messages over channels.
    - For example, to enable communication between your Raspberry Pi and the Pico board, first create a communication channel, for instance, named “talking_channel”. Create a publisher on the RPi to publish messages on this channel, and establish a listener on the Pico board to listen messages from it. That's LCM in a nutshell.

The messages that are passed are created based on the [LCM message type](https://lcm-proj.github.io/lcm/content/tutorial-lcmgen.html), which is a data structure that can include primitive data types. Here’s an example of an LCM type, `mbot_message_received_t`, which consists of two integers and a string:
```c
struct mbot_message_received_t{
    int64_t utime;            // Time of confirmation message creation
    int64_t creation_time;    // time of message creation
    string channel;           // name of channel
}
```

## MBot LCM Channels
In `mbot_firmware/src/mbot_channels.h`, we define **serial channels**:

```c
// These must match the channels also defined for mbot_lcm_serial in mbot_lcm_base
enum message_topics{
    MBOT_TIMESYNC = 201,
    MBOT_ODOMETRY = 210,
    MBOT_ODOMETRY_RESET = 211,
    MBOT_VEL_CMD = 214,
    MBOT_IMU = 220,
    MBOT_ENCODERS = 221,
    MBOT_ENCODERS_RESET = 222,
    MBOT_MOTOR_PWM_CMD = 230,
    MBOT_MOTOR_VEL_CMD = 231,
    MBOT_MOTOR_VEL = 232,
    MBOT_MOTOR_PWM = 233,
    MBOT_VEL = 234
};
```

In `mbot_lcm_base/mbot_lcm_serial/include/mbot_lcm_serial/lcm_config.h`, we define **serials channels** and **LCM channels**:

```c
/////// LCM channels //////
#define MBOT_TIMESYNC_CHANNEL "MBOT_TIMESYNC"
#define MBOT_ODOMETRY_CHANNEL "MBOT_ODOMETRY"
#define MBOT_ODOMETRY_RESET_CHANNEL "MBOT_ODOMETRY_RESET"
#define MBOT_MOTOR_PWM_CMD_CHANNEL "MBOT_MOTOR_PWM_CMD"
#define MBOT_MOTOR_PWM_CHANNEL "MBOT_MOTOR_PWM"
#define MBOT_MOTOR_VEL_CMD_CHANNEL "MBOT_MOTOR_VEL_CMD"
#define MBOT_MOTOR_VEL_CHANNEL "MBOT_MOTOR_VEL"
#define MBOT_VEL_CMD_CHANNEL "MBOT_VEL_CMD"
#define MBOT_VEL_CHANNEL "MBOT_VEL"
#define MBOT_IMU_CHANNEL "MBOT_IMU"
#define MBOT_ENCODERS_CHANNEL "MBOT_ENCODERS"
#define MBOT_ENCODERS_RESET_CHANNEL "MBOT_ENCODERS_RESET"
#define MBOT_APRILTAG_ARRAY_CHANNEL "MBOT_APRILTAG_ARRAY"

/////// serial channels //////
enum message_topics{
    MBOT_TIMESYNC = 201,
    MBOT_ODOMETRY = 210,
    MBOT_ODOMETRY_RESET = 211,
    MBOT_VEL_CMD = 214,
    MBOT_IMU = 220,
    MBOT_ENCODERS = 221,
    MBOT_ENCODERS_RESET = 222,
    MBOT_MOTOR_PWM_CMD = 230,
    MBOT_MOTOR_VEL_CMD = 231,
    MBOT_MOTOR_VEL = 232,
    MBOT_MOTOR_PWM = 233,
    MBOT_VEL = 234,
    MBOT_APRILTAG_ARRAY = 235
};
```

### Explanation
- `mbot_firmware` will be flashed onto the Pico board. The Pico communicates with the RPi over the USB cable using **serial channels**.
- `mbot_lcm_base` is installed on the RPi. On the RPi, a serial server decodes the messages received from the Pico and publishes them as LCM messages over **LCM channels**, also serializes the messages send to Pico.
- **Serial channels** are used for communication between the Pico and the RPi, while **LCM channels** handle communication between different programs on the RPi.

For example:
- In `mbot_firmware`, the file `mbot_classic.c` publishes data to **serial channels** using `comms_write_topic(MBOT_ENCODERS, &mbot_encoders);`.
- On the RPi, `lcm_serial_server.main.c` publishes the same data to **LCM channels** using `mbot_lcm_msgs_mbot_encoders_t_publish(lcmInstance, MBOT_ENCODERS_CHANNEL, &to_send);`.

The reason for using `comms_write_topic()` is that the firmware on the Pico does not support LCM directly. Messages must be serialized on the Pico and sent via USB. The serial server on the RPi receives these serialized messages, deserializes them, and republishes them over LCM. This way, any program on the RPi that requires encoder data can simply listen to the appropriate LCM channel.

The process is reversed when the RPi needs to send commands to the Pico, where the RPi serializes messages and the Pico deserializes them.

### Serial channels
> Used for communication between the RPi and Pico, defined in both `mbot_firmware` and `mbot_lcm_base`.

`message_topics` is an enumeration (enum) in C, which assigns names to integer constants. These names make the code more readable and easier to maintain. For example, in `message_topics`, we assign names to integers such as `MBOT_ENCODERS = 221`.

Why do we need these numbers, and when are they used?

In `mbot_firmware`, the Pico uses these integers to identify the serial channels for publishing or receiving messages. For example, in the code below, we publish an odometry message to the serial channel `MBOT_ODOMETRY`:

```c
comms_write_topic(MBOT_ODOMETRY, &mbot_odometry);
```
Each serial channel has a unique integer identifier to distinguish it from others. Why not just give them "real name" in string? These integers are used because they provide better performance in terms of memory usage, processing speed, and transmission efficiency—especially important given the limited resources of the Pico.

### LCM channels
> Used for communication between programs on the RPi, defined only in `mbot_lcm_base` and other RPi-based modules like `mbot_autonomy`.

Unlike serial channels, LCM channels are defined using the `#define` directive, such as `#define MBOT_VEL_CHANNEL "MBOT_VEL"`. This is done for maintainability—if the channel name changes later (e.g., from `"MBOT_VEL_CMD"` to `"MBOT_MOTOR_VEL_CMD"`), you only need to update the `#define` statement in one place instead of across the entire codebase.

Thus, it’s best practice to use `MBOT_VEL_CHANNEL` throughout the code rather than `"MBOT_VEL"`, even though they represent the same thing.


## How to send messages between RPi and Pico
> In this section, we use sending message from RPi to Pico as an example.

### Subscriber
Under the `mbot_firmware/src` directory, the `mbot_comms.c` file contains the firmware communication code. The `register_topics()` function is where all LCM subscribers and publishers are registered.

To set up a subscriber, you can utilize `register_topics()` to add new callback functions, but that's for the next section, let's just use what is already existed as an example here:

When you look at `register_topics()` in the codebase, there is a lot going on, but we only care about this one line:

```c
comms_register_topic(MBOT_VEL_CMD, sizeof(serial_twist2D_t), (Deserialize)&twist2D_t_deserialize, (Serialize)&twist2D_t_serialize, (MsgCb)mbot_vel_cmd_cb);
```
This registers a subscriber for a channel named `MBOT_VEL_CMD`. When a new message is published on `MBOT_VEL_CMD`, the callback function `mbot_vel_cmd_cb` gets triggered.

Now look at the function `mbot_vel_cmd_cb`:

```c
void mbot_vel_cmd_cb(serial_twist2D_t *msg){
    memcpy(&mbot_vel_cmd, msg, sizeof(serial_twist2D_t));
    drive_mode = MODE_MBOT_VEL;
}
```
It copies the received `serial_twist2D_t` message to the global `mbot_vel_cmd` variable. The `mbot_loop()` function under `mbot_classic.c` then utilizes this variable:
```c
else if(MBOT_DRIVE_TYPE == DIFFERENTIAL_DRIVE){
  mbot_motor_vel_cmd.velocity[MOT_L] = (mbot_vel_cmd.vx - DIFF_BASE_RADIUS * mbot_vel_cmd.wz) / DIFF_WHEEL_RADIUS;
  mbot_motor_vel_cmd.velocity[MOT_R] = (-mbot_vel_cmd.vx - DIFF_BASE_RADIUS * mbot_vel_cmd.wz) / DIFF_WHEEL_RADIUS;
}
```
In this code snippet, the `mbot_vel_cmd` variable's linear velocity (`vx`) and angular velocity (`wz`) components are being used to calculate the target velocities for the left and right motors of a differential drive robot,  the firmware is ready to process the incoming message. To make the robot move, we need to publish the desired velocities to the `MBOT_VEL_CMD` channel.

Next, let's discuss the structure of the message.

### LCM message
From the code snippet discussed earlier, we know the subscriber takes in data type `serial_twist2D_t`. This is the serialized form of `twist2D_t`, automatically generated for use with `mbot_firmware`. So the actual LCM type that we defined is `twist2D_t`. To check the structure of it, you can run `mbot lcm-msg show twist2D_t` in the terminal, details [here](/docs/botlab/how-to-guide/mbot-cli-tools).

The message structure looks like this:
```
struct twist2D_t{
    int64_t utime;
    float vx;
    float vy;
    float wz;
}
```

### Publisher
Now that we understand the message we wish to publish, let's create the publisher.

Below is a simple Python example:
```python
import lcm
from mbot_lcm_msgs.twist2D_t import twist2D_t

# Create the velocity command message
msg = twist2D_t()
# Assign values to the message
msg.vx = 10
msg.wz = 10
# Publish the velocity command
lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")
lc.publish("MBOT_VEL_CMD", msg.encode())
```

When developing, ensure your code resets the velocities to 0 after completion of the logic.

The line `lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")` configures UDP multicast communication, specifying the multicast address, port, and TTL for the packets. This URL is the default setting. For more on UDP Multicast Setup, see [LCM's multicast setup documentation](https://lcm-proj.github.io/lcm/content/multicast-setup.html).

At this point, your communication setup is complete. Once you run the Python file, the LCM message will be published to the "MBOT_VEL_CMD" channel, serialized, sent to the Pico over the USB cable, deserialized, and finally used to drive the MBot.


## Add new LCM message types
### Communication on Pi only
> When you want to send data from one program to another when both of them running on the Pi.

Under editting...

### Communication between Pi and Pico
> When you want to grab data from the Pico to the Pi, or you want to send data to the Pico.

1. Define the data structure.
    - All LCM message types are stored in the `mbot_lcm_base/mbot_msgs/lcmtypes` directory. To add a new LCM type, create your custom LCM message `mbot_example_t.h` in this location.
2. Define the channel
    - Messages are sent over channels. Since the message needs to be sent between the RPi and Pico, you must define both LCM channels and serial channels, and the definition should be all the same cross files:
      - In `mbot_lcm_base/mbot_lcm_serial/include/mbot_lcm_serial/lcm_config.h`, define serial channels and LCM channels.
      - In `mbot_firmware/src/mbot_channels.h` define serial channels.
        ```c
        #define MBOT_EXAMPLE_CHANNEL "MBOT_EXAMPLE"

        enum message_topics{
          ...
          MBOT_EXAMPLE = 200
        };
        ```

    Notice: The channel number is limited to a maximum of 255 due to the 8-bit limit. The numbers 0 and 255 might reserved for special purposes. Therefore, 254 is your highest usable value for channel identifiers.
3. Register the channel in `mbot_lcm_base/mbot_lcm_serial/src/lcm_serial_server_main.c`
  - Include the new message by add `#include <mbot_lcm_msgs_mbot_example_t.h>`
  - register the topic (say if setting a subscriber):
    ```c
    comms_register_topic(MBOT_EXAMPLE, sizeof(serial_mbot_example_t), (Deserialize)&mbot_example_t_deserialize, (Serialize)&mbot_example_t_serialize, (MsgCb)serial_example_msg_t_cb);
    ```
  - Add the callback function:
    ```c
    void serial_example_msg_t_cb(serial_mbot_example_t* data){
      mbot_lcm_msgs_mbot_example_t to_send = {0};
      // populate to_send then publish it to the LCM
      mbot_lcm_msgs_mbot_example_t_publish(lcmInstance, MBOT_EXAMPLE_CHANNEL, &to_send);
    }
    ```
4. Make and install in `mbot_lcm_base` to apply new changes
    - Add the new defined type to `mbot_lcm_base/mbot_msgs/CMakeLists.txt`:
      ```cmake
      set(LCM_FILES
        ...
        lcmtypes/slam_status_t.lcm
        lcmtypes/exploration_status_t.lcm
        ...
        lcmtypes/mbot_example_t.lcm
      )
      ```
    - Then build and install the updated LCM message types:
      ```bash
       $ cd ~/mbot_ws/mbot_lcm_base
       $ ./scripts/install.sh
      ```
5. Modify the `mbot_firmware` to register the new channel and set up publish/subscribe functions in `src/mbot_comms.c`.
  - Still under `register_topics()` (say if setting a publisher):
    ```c
    comms_register_topic(MBOT_EXAMPLE, sizeof(serial_mbot_example_t), (Deserialize)&mbot_example_t_deserialize, (Serialize)&mbot_example_t_serialize, NULL);
    ```
  - After making the necessary changes, recompile and flash the updated firmware to the Pico.
6. Now your newly defined lcm message is ready to use.
  - To publish from the firmware side:
    ```c
    comms_write_topic(MBOT_EXAMPLE,, &mbot_example);
    ```
  - To publish/subscribe from the Pi side:

    The following code from `mbot_autonomy` demonstrates how to subscribe to and publish messages:
    ```cpp
     lcm_.subscribe(PATH_REQUEST_CHANNEL, &MotionPlannerServer::handleRequest, this);
     lcm_.publish(CONTROLLER_PATH_CHANNEL, &path);
    ```
    - `lcm_.subscribe` listens for messages on PATH_REQUEST_CHANNEL. When a message is received, it triggers the callback function `handleRequest`. The `this` pointer refers to the current instance of the class.
      - `The MotionPlannerServer::handleRequest` callback function is where we define the preferred actions to take when data arrives.
    - `lcm_.publish` sends the `path` message on CONTROLLER_PATH_CHANNEL.
