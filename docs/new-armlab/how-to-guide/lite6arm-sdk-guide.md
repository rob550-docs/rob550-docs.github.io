---
layout: default
title: Lite 6 Arm & SDK Guide
nav_order: 5
grand_parent: New Armlab
parent: How-to Guide
last_modified_at: 2026-09-02 12:00:00 -0400
---

> The arm at your station is a **UFACTORY Lite 6**, and you talk to it over the network with the **xArm Python SDK**. This page summarizes the hardware limits you have to design around and the SDK calls you will actually use.

{: .note}
In this lab you will normally go through the wrapper in `src/lite6arm.py` rather than calling the SDK directly. Read this page anyway — the wrapper is thin, and every error message you see comes from the layer described here.

### Contents
* TOC
{:toc}

## The arm

The Lite 6 is a 6-DOF collaborative arm with the control electronics built into its base. Its specifications, joint limits and payload are on the [Hardware page](/docs/new-armlab/hardware#ufactory-lite-6-arm) — this page covers only how you talk to it.

Two hardware facts drive most of the software problems you will hit:

- **The joint limits are asymmetric**, and J5 (±124°) and J3 (−3.5° to 300°) are tight. A mathematically valid IK solution can still be unreachable, which surfaces as error 23 in the [error table](#errors).
- **The payload is 600 g including the gripper**, so a grasp can fail for reasons that have nothing to do with your code.

## Connecting

The arm is on the network, not USB. The SDK connects to an IP address:

```python
from xarm.wrapper import XArmAPI

arm = XArmAPI('192.168.1.xxx')   # the IP on the label at the back of your arm
```

This is the address you set as `XARM_IP` during [setup](/docs/new-armlab/setup-guide#4-set-your-stations-arm-ip).

{: .note}
If the connection fails, `ping` the address from a terminal before touching your code. No reply means a cable, IP or arm problem, not a Python problem. See [Linux Command Line Tools](/docs/new-armlab/how-to-guide/linux-clt#checking-hardware-and-the-network).

## The startup sequence

The arm will not move until you have enabled motion, chosen a mode, and put it in a ready state. This three-step sequence is the single most important thing on this page:

```python
arm.motion_enable(enable=True)
arm.set_mode(0)     # position control
arm.set_state(0)    # ready to move
```

{: .important}
You must repeat this sequence **every time you clear an error**, and after an emergency stop. Clearing the error alone does not make the arm ready again — a very common "my code stopped working and I don't know why" bug.

## Modes

`set_mode()` selects how the arm interprets commands.

| Mode | Meaning | Use in this lab |
| ---- | ------- | --------------- |
| 0 | Position control | **The default.** Point-to-point moves |
| 1 | Servo motion | High-frequency small steps, streamed by you |
| 2 | Joint teaching | **Free-drive.** Motors release so you can move the arm by hand |
| 3 | Cartesian teaching | Not available |
| 4 | Joint velocity control | Velocity commands per joint |
| 5 | Cartesian velocity control | Velocity commands on the TCP |
| 6 | Joint online trajectory planning | Retarget a joint move while it runs |
| 7 | Cartesian online trajectory planning | Retarget a Cartesian move while it runs |

{: .highlight}
**Mode 2 is how you do teach-and-repeat.** Put the arm in joint teaching mode, physically guide it to a pose, read the joint angles back with `get_servo_angle()`, and store them. Switch back to mode 0 to replay them.

{: .warning}
In mode 2 the arm is back-driveable, so **support it before you switch** — an arm holding a pose will sag when the motors release.

Mode 1 is a trap for beginners: it executes each command immediately at maximum speed, so it only behaves sensibly if you are streaming small increments at a high, steady rate. Stay in mode 0 unless you know you need otherwise.

## States

Confusingly, the values you **write** with `set_state()` are not the same as the values you **read** back from `get_state()`.

| `set_state(n)` | Meaning |
| -------------- | ------- |
| 0 | Ready to move |
| 3 | Pause |
| 4 | Stop |

| `get_state()` returns | Meaning |
| --------------------- | ------- |
| 1 | Moving |
| 2 | Idle / sleeping |
| 3 | Paused |
| 4 | Stopped |

So after a successful `set_state(0)`, a healthy idle arm reads back as state **2**, not 0.

## Units and conventions

{: .warning}
**The SDK works in millimetres and degrees by default.** Not metres, not radians. Mixing units is the most common source of bugs in this lab, and a factor-of-1000 error will drive the arm into the board.

- Cartesian positions: `x`, `y`, `z` in **mm**
- Orientation: `roll`, `pitch`, `yaw` in **degrees**
- Joint angles: **degrees**
- Speeds: mm/s for Cartesian moves, °/s for joint moves

Most methods take an `is_radian` argument, and you can set the default for the whole session when you construct the object:

```python
arm = XArmAPI('192.168.1.xxx', is_radian=True)
```

Pick one convention and apply it consistently. If your kinematics code works in radians — and it probably should — decide early whether you convert at the boundary or set `is_radian=True` everywhere, and write it down for your teammates.

## Reading the arm's state

Both getters return a `(code, value)` tuple — the code first, then the data:

```python
code, pose = arm.get_position()      # [x, y, z, roll, pitch, yaw]
code, angles = arm.get_servo_angle() # [j1, j2, j3, j4, j5, j6]
```

{: .note}
Forgetting that these return a tuple, and treating the result as a plain list, produces a confusing error one or two lines later. Unpack both values.

## Commanding motion

### Joint space

```python
arm.set_servo_angle(angle=[0, 20, 30, 0, 40, 0], speed=30, wait=True)
```

### Cartesian space

```python
arm.set_position(x=200, y=0, z=150, roll=180, pitch=0, yaw=0,
                 speed=100, wait=True)
```

### Go home

```python
arm.move_gohome(wait=True)
```

Common arguments:

| Argument | Meaning |
| -------- | ------- |
| `speed` | mm/s for Cartesian moves, °/s for joint moves |
| `mvacc` | Acceleration limit |
| `wait` | `True` blocks until the move finishes; `False` returns immediately |
| `relative` | Treat the position as an offset from the current one |

{: .important}
`wait=True` is what you want while you are learning — without it your next command is issued while the arm is still moving, and a sequence of waypoints will blur together or be dropped. Use `wait=False` deliberately, not by accident.

## The gripper

The gripper is custom-built for this lab (see [Hardware](/docs/new-armlab/hardware#gripper--end-effector)) but is driven by the standard Lite 6 gripper calls:

```python
arm.open_lite6_gripper()
arm.close_lite6_gripper()
arm.stop_lite6_gripper()    # de-energize and hold where it is
```

Each returns a code like any other SDK call. `open` and `close` drive the gripper; `stop` cuts the drive.

{: .warning}
The gripper is **binary** — no width, no force, no feedback about whether it grasped anything. Your state machine has to verify a grasp some other way, such as looking at it with the camera, and should aim for a repeatable approach pose rather than a closed loop on grip force.

{: .note}
Give the gripper time to finish moving before the arm drives away. The call returns as soon as the command is sent, not when the jaws have finished travelling.

## Errors

**Every SDK call returns a code.** `0` means success; anything else means the command did not do what you asked. Ignoring return codes turns a clear failure into a mysterious one.

| Code | Meaning |
| ---- | ------- |
| 0 | Success |
| 1 | There are errors that have not been cleared |
| 2 | There are warnings that have not been cleared |
| 9 | State is not ready to move |
| −2 | Arm not ready — motion not enabled, or state not set |
| −9 | Emergency stop active |

To inspect and clear a controller error:

```python
code, (err, warn) = arm.get_err_warn_code()
arm.clean_error()
arm.clean_warn()

# the arm is NOT ready yet — redo the startup sequence
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(0)
```

Controller error codes worth recognizing:

| Error | Meaning |
| ----- | ------- |
| 1, 2, 3 | Emergency stop (button, control-box IO, three-state switch) |
| 10 | Servo motor error |
| 21 | Kinematics solution failed — the pose you asked for has no IK solution |
| 22 | Self-collision detected |
| 23 | Joint angle exceeds limit — check against the joint limit table above |
| 24 | Commanded speed exceeds the maximum |
| 31 | Abnormal current, consistent with a collision |
| 35 | Safety boundary limit triggered |

{: .note}
Errors 21 and 23 are the ones you will meet while writing IK. Both mean the target was unreachable, not that the arm is broken.

## Reference

- [xArm Python SDK on GitHub](https://github.com/xArm-Developer/xArm-Python-SDK) — source, and the `example/wrapper/lite6/` folder of runnable examples
- [Full API documentation](https://github.com/xArm-Developer/xArm-Python-SDK/blob/master/doc/api/xarm_api.md) — every method on `XArmAPI`
- [API return and error codes](https://github.com/xArm-Developer/xArm-Python-SDK/blob/master/doc/api/xarm_api_code.md)
- [UFACTORY developer documentation](https://docs.api.ufactory.cc/)
- [Lite 6 product page and specifications](https://www.ufactory.us/product/lite-6)
- [Lite 6 user manual (PDF)](https://www.ufactory.cc/wp-content/uploads/2023/05/Lite6-User-Manual-V2.0.0.pdf)
