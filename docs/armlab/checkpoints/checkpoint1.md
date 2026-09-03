---
layout: default
title: Checkpoint 1
nav_order: 1
parent: Checkpoints
grand_parent: Armlab
last_modified_at: 2026-09-02 12:00:00 -0400
---

This checkpoint gets you moving the arm and reasoning about where things are in space. You will start in the arm's own web interface — the quickest way to drive the hardware and see what it can do — then move to the control station codebase you will spend the rest of the lab in. By the end you will have made the arm repeat a task you taught it by hand, and built your first camera-to-world transform with nothing but a tape measure.

### Contents
* TOC
{:toc}

## Before you start

- Your station is set up and the control station runs. If not, work through the [Setup Guide](/docs/armlab/setup-guide) first.
- You know where the emergency stop is. It is on the table next to the arm.
- You have read the [Hardware](/docs/armlab/hardware) page, in particular the joint numbering, the rotation directions and the joint limits.

The board the arm is mounted on has a **50 mm × 50 mm grid**. The frame everything in this lab is expressed in is defined like this:

- the **origin** is the centre of the arm's mounting location, which sits on grid lines
- the **board surface is `z = 0`**
- positions are in **millimetres**

Because the origin is on a grid line, every grid intersection is a whole multiple of 50 mm away from it — which is what makes the board a usable ruler.

{: .warning}
The gripper can reach the board. When you are driving the arm by hand, watch the **z** readout and keep the tool above the surface — it is very easy to drive the end effector straight into the board while concentrating on x and y.

## Task 1.1  Drive the arm from the web interface

The Lite 6 serves its own web interface, **UFACTORY Studio**. Point a browser at your arm's IP address on port **18333** — the address is the one you set as `XARM_IP` during setup, printed on the label on the back of the arm:

```
http://192.168.1.xxx:18333
```

There is no login.

TODO: screenshot of the UFACTORY Studio landing page.

**Instructions**

1. Find the **joint jog** controls and move each of the six joints one at a time with the sliders. Watch which physical joint moves, and compare the direction of travel to the joint numbering on the [Hardware](/docs/armlab/hardware#ufactory-lite-6-arm) page.
2. **Test the emergency stop.** With the arm somewhere away from its home position, press the E-stop on the table next to the arm. The arm stops immediately, and stays exactly where it was — locks engage on the joints, so it does not drop or sag. Release the E-stop and bring the arm back under control before carrying on.
3. Push a joint gently toward its limit and watch what the interface reports. J5 (±124°) and J3 (−3.5° to 300°) will stop you long before the others.
4. Switch to the **Cartesian** controls and move the tool along x, y and z in the base frame. Notice that the joints all move together to produce one straight-line motion of the tool.
5. Watch the **end-effector position readout** as you move. Get a feel for which numbers change when you move in each direction.
6. Lower the gripper to the height where it can grasp a block resting on the board, and **write that z value down.** You will need it in Task 1.2. It is not zero: the readout reports the tool reference point, which is not the same place as the tips of the gripper fingers.
7. Do a manual pick and place: drive the tool over a block, lower it, close the gripper, lift, move to a new spot, and open the gripper. Write down the pose you used to grasp the block.
8. **Measure your blocks.** You need the height to work out how tall a stack of two or three will be, and the width to know the gripper can take them.

{: .note}
Hitting the E-stop is safe for the arm. Locks engage on the joints and hold it in place, so nothing falls and nothing is damaged — get comfortable using it.

**Hints**

- Reduce the speed before you go near the board.
- Approach a block from directly above rather than from the side. You will use the same strategy in code later.
- The gripper is open or closed, nothing in between, and it cannot tell you whether it actually grabbed anything. Look at it.

{: .sanity_check}
You have moved a block from one place on the board to another using only the web interface, and you have written down two numbers you will need next: the z at which the gripper grasps a block on the board, and the block dimensions.

## Task 1.2  Script the arm from the web interface

The web interface has a built-in Python editor with a set of example scripts. Open one of the examples, read it, and use it as your starting point — it will show you the connection and motion calls the arm expects.

TODO: screenshot of the Python editor with an example loaded, and the name of the example that makes the best starting point.

**Instructions**

1. Place three blocks in front of the robot at three grid positions of your choosing. Write the positions down in millimetres in the base frame.
2. Write a script that **stacks** the three blocks into a single tower at a fourth position.
3. Extend it to **unstack** them — return each block to its original position.
4. Test it. Fix the parts that do not work.
5. Wrap the whole stack-and-unstack cycle in a loop so that, in principle, it runs forever.
6. Leave it running and watch it for a while.

**Hints**

- The 50 mm grid is your friend. Choose positions on grid intersections so that you can measure and re-measure them.
- Give yourself an approach height above each block, then descend, then grip. Going straight to the grasp pose invites a collision with the block you are trying to pick.
- Use the grasp height you measured in Task 1.1 rather than guessing. A tower of three blocks is taller than one, so the place height has to step up by one block height each time — that is what you measured the blocks for.
- Give the gripper time to finish moving before the arm drives away.

{: .highlight}
Run the loop long enough and it will eventually fail, even though nothing in the code changed. Watch for how it fails and think about why: small positioning errors accumulate in the stack, blocks get nudged, and there is nothing in this system that can notice. Nothing here senses the world — that is the gap the rest of the lab closes.

{: .sanity_check}
Your script stacks and unstacks three blocks repeatedly without intervention.

## Task 1.3  A waypoint follower in the control station

Now switch to the codebase you will use for the rest of the lab. Start the control station and spend a few minutes with the interface: the video panel and its four views, the joint readouts and Direct Control. Read [Software](/docs/armlab/software) alongside it so you know which file each panel talks to.

**Instructions**

1. Open `src/state_machine.py` and find the waypoint handlers. They are stubs.
2. Implement a waypoint follower: given a list of joint-space configurations, drive the arm through them in order.
3. Give it a list of waypoints of your own choosing so that the arm performs a short, deliberate routine — make it a dance if you like. Six to ten waypoints is plenty.
4. Trigger it from the GUI and watch it run.

**Hints**

- Joint angles in this codebase are **radians**. The GUI converts to degrees for display only — see [Units and conventions](/docs/armlab/software#units-and-conventions).
- Check every waypoint against the joint limits before you send it. An unreachable configuration will be refused by the arm, not silently clamped.
- The arm needs time to reach each waypoint. Decide how you know a motion is finished before starting the next one.
- Start slow. The speed slider scales every motion command.

{: .sanity_check}
Clicking the button runs your routine from start to finish and the arm ends where you expect.

## Task 1.4  Teach and repeat

Teaching by demonstration is how a lot of industrial arms get programmed: you move the robot through the motion by hand, and it plays it back. You are going to build that, and then use it to teach the stacking task from Task 1.2.

**Instructions**

1. Put the arm into **teach mode** from the control station — the Manual Mode toggle calls `set_teach_mode()`. This arms hand-guiding, but does not start it.
2. **Press the button on the arm.** That is what actually releases it: the arm now holds its own weight while staying easy to push around, so you can guide it by hand.
3. Add a **record** control to the control station that captures the arm's current joint configuration as a waypoint.
4. Add a **playback** control that replays the recorded waypoints in order.
5. Add a way to insert **open gripper** and **close gripper** actions into the sequence, so a recorded plan can grip and release, not just move.
6. Use it to teach the robot the **stacking** task — three blocks into a tower. Unstacking is not required.

**Hints**

- A waypoint needs to carry the gripper state as well as the joint angles. Two consecutive waypoints can share a pose and differ only in whether the gripper is closed — that is exactly what a grasp looks like.
- Decide what happens on playback when a waypoint changes the gripper: the arm should finish moving, then actuate, then move on.
- The Waypoint Recorder card in the GUI already has buttons wired to state-machine states. You can use them, or add your own.
- Teach a few waypoints and play them back before you try to teach the whole stacking sequence.

{: .sanity_check}
You can hand-guide the arm, record a sequence that includes gripper actions, and have it stack three blocks on playback without you touching it.

## Task 1.5  Measure the camera extrinsics

The camera sees pixels. The arm works in millimetres in its own base frame. The **extrinsic matrix** is what connects them: a homogeneous transform describing where the camera sits relative to the robot.

Later in the lab you will compute this automatically from AprilTags. First you are going to measure it by hand, with a tape measure, so that you know exactly what the numbers in that matrix mean.

**Instructions**

1. Measure the position of the camera relative to the robot base frame: how far along x, y and z its optical centre sits.
2. Work out its orientation — which way the camera's axes point relative to the base frame.
3. Build the 4 × 4 homogeneous transform from those measurements. Be explicit about **which direction** it maps: world into camera, or camera into world.
4. Get the camera intrinsic matrix. `camera.py` reads it from the camera at startup and prints it:
    ```bash
    cd src
    python camera.py
    ```
5. Now test it. For each of these four points on the board, at `z = 0` in the base frame:

    | Point | x (mm) | y (mm) |
    | ----- | ------ | ------ |
    | A | 0 | 200 |
    | B | 0 | −200 |
    | C | 200 | 200 |
    | D | 200 | −200 |

    - **Predict** the pixel each one should land on, using your extrinsic matrix and the intrinsic matrix.
    - **Measure** where it actually lands: start the control station, hover the mouse over that point in the video, and read the pixel coordinate from the readout under the image.
    - Record both, and the difference.

**Hints**

- Draw the two coordinate frames before you write down any numbers. Most of the difficulty here is bookkeeping, not arithmetic.
- A point in the world becomes a pixel in two steps: world → camera frame using the extrinsics, camera frame → pixel using the intrinsics. Keep them separate and you can debug them separately.
- Remember the camera looks *down* at the board. Its z axis points roughly along the board's −z.
- Your numbers will not match well. That is the point — quantify how badly, and think about why.

{: .sanity_check}
You have predicted and measured pixel coordinates for all four points, and you can explain the sign and rough size of the disagreement.

## Deliverables

Submit the following on Canvas.

{: .submission}
**1)** A short video of the arm replaying your taught sequence from Task 1.4, gripper actions included. <br>
**2)** Your recorded waypoint list from Task 1.4 as a table — all six joint angles plus the gripper state for each waypoint, **with units stated**. <br>
**3)** Two or three sentences on how you represented a waypoint, and why the gripper state is stored with the pose rather than in a separate list. <br>
**4)** A labelled sketch of your Task 1.5 setup: the robot base frame, the camera frame, both sets of axes, and the distances you measured. <br>
**5)** Your 4 × 4 extrinsic matrix with real numbers, and a statement of which direction it maps. <br>
**6)** A table for the four board points: predicted pixel, measured pixel, and the difference for each. <br>
**7)** Two or three sentences on where the error comes from and how large you expected it to be.

{: .required_for_report}
From this checkpoint, carry the following into your final report: <br>
**1)** The labelled frame diagram from Task 1.5. <br>
**2)** The extrinsic matrix and how you arrived at it. <br>
**3)** The predicted-vs-measured pixel table for the four board points. <br>
**4)** Your analysis of the error — its sources, its size, and what it implies about calibrating this way.
