---
layout: default
title: LCM for Beginners
parent: Staff Guide
nav_order: 5
last_modified_at: 2024-03-17 14:20:48 -0500
---

> This guide introduces LCM system to developers working on a differential drive MBot, specifically targeting absolute beginners.

Assume that you have finished all the steps under [botlab setup guide](/docs/botlab/setup-guide/mbot-system-setup). In this post, we'll focus on an example of sending desired velocity commands from the Jetson Nano to the control board to make the MBot move using LCM.

### Contents
- TOC
{:toc}

## What is LCM?
[LCM](https://lcm-proj.github.io/lcm/) stands for Lightweight Communications and Marshalling. It's about sending data between programs or computers (Communications) and getting data ready for sending or storing (Marshalling). 
- If you know ROS, LCM works similarly with the same publish-subscribe model. LCM types are like ROS messages, and LCM channels are like ROS topics.
- If ROS is new to you, think of LCM as a way to send and receive messages over channels.
    - To enable communication between your Jetson and the Control board, first create a communication channel, for instance, named “talking_channel”. Create a publisher on the Jetson to publish messages on this channel, and establish a subscriber on the Control board to receive messages from it. That's LCM in a nutshell.

The messages that are passed are created based on the [LCM message type](https://lcm-proj.github.io/lcm/content/tutorial-lcmgen.html), which is a data structure that can include primitive data types. Here’s an example of an LCM type, `mbot_message_received_t`, which consists of two integers and a string:
```
struct mbot_message_received_t{
    int64_t utime;            // Time of confirmation message creation
    int64_t creation_time;    // time of message creation  
    string channel;           // name of channel 
}
```

## Quick Start
### Subscriber
Under the `mbot_firmware/src` directory, the `mbot.c` file contains the firmware C code. The `register_topics()` function is where all LCM subscribers and publishers are registered.

Remember, a subscriber listens, and a publisher publishes. We talk to control board by creating a publisher on the Jetson and publishing to a channel that a subscriber on the control board listens to.

To set up a subscriber, you can utilize the existing setup in `register_topics()`. You can also create your customized message and new channel, but off a quick start, let's just use what is already on the control board:

```c
void register_topics(){
    ...
    comms_register_topic(MBOT_MOTOR_PWM_CMD, sizeof(serial_mbot_motor_pwm_t), (Deserialize)&mbot_motor_pwm_t_deserialize, (Serialize)&mbot_motor_pwm_t_serialize, (MsgCb)mbot_motor_vel_cmd_cb);
    comms_register_topic(MBOT_MOTOR_VEL_CMD, sizeof(serial_mbot_motor_vel_t), (Deserialize)&mbot_motor_vel_t_deserialize, (Serialize)&mbot_motor_vel_t_serialize, (MsgCb)mbot_motor_pwm_cmd_cb);
    comms_register_topic(MBOT_VEL_CMD, sizeof(serial_twist2D_t), (Deserialize)&twist2D_t_deserialize, (Serialize)&twist2D_t_serialize, (MsgCb)mbot_vel_cmd_cb);
}
```

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
It copies the received `serial_twist2D_t` message to the global `mbot_vel_cmd` variable. The `mbot_loop()` function then utilizes this variable:
```c
else if(MBOT_DRIVE_TYPE == DIFFERENTIAL_DRIVE){
    mbot_motor_vel_cmd.velocity[params.mot_left] = (mbot_vel_cmd.vx - DIFF_BASE_RADIUS * mbot_vel_cmd.wz) / DIFF_WHEEL_RADIUS;
    mbot_motor_vel_cmd.velocity[params.mot_right] = (-mbot_vel_cmd.vx - DIFF_BASE_RADIUS * mbot_vel_cmd.wz) / DIFF_WHEEL_RADIUS;
    ...
}
```

In this code snippet, the `mbot_vel_cmd` variable's linear velocity (`vx`) and angular velocity (`wz`) components are being used to calculate the target velocities for the left and right motors of a differential drive robot, so it translates a desired linear and angular velocity into specific motor speeds.

Now we know that the control board already has a subscriber listening to `MBOT_VEL_CMD` channel, ready to process the incoming message. To make the robot move, we need to publish the desired velocities to the `MBOT_VEL_CMD` channel. 

Next, let's discuss the structure of the message.

### LCM message
From the code snippet discussed earlier, we know the subscriber takes in data type `serial_twist2D_t`. This is the serialized form of `twist2D_t`, automatically generated for use with `mbot_firmware`. So the actual LCM type that we defined is `twist2D_t`. It is defined in the `mbot_lcm_base/mbot_msgs/lcmtypes` directory, where all LCM message definitions are stored.

Inside that directory, you'll find `twist2D_t.lcm`:
```
struct twist2D_t{
    int64_t utime;
    float vx;
    float vy; // this should be 0 when use differential drive 
    float wz;
}
```
For our purposes, `vx` and `wz` are the relevant parameters.

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

When developing, ensure your code resets the velocities to 0 after completion of the logic. Failing to do so may result in the wheels continuing to run, even after the program has been terminated.

The line `lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")` configures UDP multicast communication, specifying the multicast address, port, and TTL for the packets. This URL is the default setting. For more on UDP Multicast Setup, see [LCM's multicast setup documentation](https://lcm-proj.github.io/lcm/content/multicast-setup.html).

