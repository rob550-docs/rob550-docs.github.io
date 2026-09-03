---
layout: default
title: Checkpoint 2
nav_order: 2
parent: Checkpoints
grand_parent: Armlab
last_modified_at: 2026-09-03 12:00:00 -0400
---

In Checkpoint 1 you measured the camera's position with a tape measure and found out how rough that answer was. This checkpoint replaces it: the AprilTags on the board have known positions, so the camera can work out where it is by itself. You will build that calibration, turn it into a pixel-to-world pipeline the whole rest of the lab depends on, and then measure honestly how good it is — not just on the board surface, but through the workspace in three dimensions.

The second half is forward kinematics. The real arm reports its own end-effector pose, so your FK is not needed there — but the simulator has no such thing, and its readout stays blank until you write one.

### Contents
* TOC
{:toc}

## Before you start

- Checkpoint 1 is done: you have calibrated intrinsics, a hand-measured extrinsic matrix, and measurements of your board.
- You know where your AprilTags sit relative to the robot base frame. If you have not measured them yet, do that first — every number in this checkpoint depends on it.

{: .important}
`TAG_WORLD_POINTS` at the top of `src/camera.py` ships with placeholder positions. Replace them with the positions you measured. A calibration solved against the wrong world points will still return a matrix, and it will be wrong.

---

# Part A — Automatic camera calibration

## Task 2.1  Draw the tag detections

Before trusting the detector, look at what it sees. `VideoThread` already runs the detector for you and leaves the results in `self.tag_detections`, refreshed every sixth frame.

**Instructions**

1. Implement `draw_tags_in_rgb_image()` in `src/camera.py`. For every detection, draw:
    - the tag's **ID**
    - a marker on its **centre**
    - its **outline**, from the four corners
2. Select the **Tags** view in the GUI to see your work.

**Hints**

- Each detection carries `tag_id`, `center` and `corners`. The corners come back in a consistent order, which is what lets you draw an outline rather than a scatter of dots.
- Draw onto a copy of `self.video_frame`, and assign the result to `self.tag_image_frame`.
- The detector runs at about 5 Hz while the video runs at 30, so your overlay will lag a fast-moving board. That is expected.

{: .sanity_check}
Move the board by hand and the outlines and IDs track the tags. If an outline is twisted or the IDs jump between tags, you are reading the corner order wrong.

## Task 2.2  Solve the extrinsics from the tags

Now compute the camera pose automatically, and put it behind the **Calibrate** button that is already wired to the `calibrate` state.

**Instructions**

1. Fill in `TAG_WORLD_POINTS` with your measured tag positions.
2. Implement `estimate_extrinsics_from_tags()`. Given tags detected in the image and their known world positions, solve for the transform between the two frames.
3. Store the result in `self.extrinsic_matrix` (world → camera) and cache the inverse in `self.extrinsic_matrix_inv`. Set `self.camera_calibrated`.
4. Return a helpful `(ok, message)` — that message is what appears in the GUI status bar.
5. Compare the matrix against the one you measured by hand in Checkpoint 1.

**Hints**

- `cv2.solvePnP` takes your known world points, the matching image points, and `self.intrinsic_matrix`, and returns a rotation vector and translation. `cv2.Rodrigues` turns the rotation vector into a matrix.
- The detector is currently called without pose estimation, so detections give you corners and centres but not tag poses. Either work from the centres, or use all four corners of every tag — which gives you sixteen points instead of four and a much better-conditioned solve. `TAG_SIZE_MM` is defined for exactly this.
- Sanity-check the answer before you trust it: the camera sits about 1 m above the board looking down, so the translation should say so.
- Be explicit with yourself about which direction your matrix maps. Storing world → camera when you meant camera → world produces results that look almost right, which is the worst kind of wrong.

{: .sanity_check}
Pressing Calibrate reports success, and the recovered camera position is within a few centimetres of where you measured it in Checkpoint 1.

## Task 2.3  From pixels to the world

The calibration is only useful once it can answer the question the rest of the lab asks constantly: *this pixel — where is it on the board?*

**Instructions**

Implement the three functions that make up the chain:

1. `depth_to_camera_point(x, y, depth_raw)` — a pixel plus a raw depth reading, to a 3D point in the **camera** frame.
2. `camera_to_world(camera_point)` — camera frame to **world** frame.
3. `image_to_world(x, y)` — the whole chain, using live depth.

**Hints**

- Raw depth units are not millimetres. Multiply by `self.depth_scale_mm`.
- Depth is already aligned to colour, so `(u, v)` means the same point in both images. That alignment is why this works at all.
- Deprojection is the intrinsic matrix run backwards: `X = (u − cx)·Z / fx`, `Y = (v − cy)·Z / fy`.
- Depth is noisy on dark, shiny or steeply angled surfaces, and returns zero where it has no reading. Decide what your code does with a zero.

{: .sanity_check}
Hover over a grid intersection you know the coordinates of. The world readout under the video should land within a few millimetres, and reading `(0, 0)` at the base of the arm.

## Task 2.4  The top-down workspace view

The camera looks at the board from an angle, so the board appears as a trapezoid. A homography maps it back to a rectangle, which makes everything downstream easier to reason about.

**Instructions**

1. Implement `_update_workspace_transform()`: build the homography from four source points in the image to four destination points forming a rectangle. Store both it and its inverse.
2. Implement `update_workspace_frame()` to warp the video into `self.workspace_frame`.
3. Implement `workspace_pixel_to_image()` so that clicking in the rectified view still maps back to the right pixel in the original image.
4. Select the **Workspace** view to check it.

**Hints**

- Source points can be the tag centres (you have those pixel coordinates already) or the board corners (you know their world coordinates and now have the means to project them into the image).
- Choose destination points that keep the board's aspect ratio. Stretching it will not break the maths but will make everything harder to look at.
- Step 3 is the one people forget. The mouse readout and Click Pick & Place both go through it, and if it is missing, clicks in the Workspace view land somewhere else entirely.

{: .sanity_check}
The board fills the Workspace view square and level, the grid lines run parallel to the edges of the frame, and hovering still reports correct world coordinates.

## Task 2.5  Measure how good your calibration is

A calibration you have not measured is a guess. This task is about producing evidence — and finding where it fails, because it will fail somewhere.

### 2.5a  Project the grid

Write a function that takes the known world coordinates of the board's grid intersections, projects them into the image through your intrinsics and extrinsics, and draws them over the video.

Where the projected points sit on the real grid lines, your calibration is good. Where they drift off, it is not. Look especially at the edges and corners of the board.

### 2.5b  Check the board plane

The board surface is `z = 0` everywhere. So sample a grid of pixels across the empty board, run each through `image_to_world`, and record the world `z` you get back. Every one of them should be zero.

Turn the results into a picture — a heat map, or contours over the board. This costs you no measuring at all and shows exactly how the error is distributed across the workspace.

### 2.5c  Check it in three dimensions

The board plane is only one slice of the workspace. To test the rest, you need a target whose true position you know at heights above the board — and the arm is a far better ruler than a tape measure.

Use the **calibration fixture**: a 3D-printed AprilTagged object the gripper grips in a known, repeatable way, so the tag's pose relative to the end-effector frame is known to a fraction of a millimetre.

**Instructions**

1. Grip the fixture.
2. Drive the arm to **four board locations × three heights** (roughly `z` = 50, 150 and 250 mm), using the jog controls or waypoints from Checkpoint 1.
3. At each pose, record two things:
    - **truth** — the arm's reported end-effector pose, plus the known fixture offset
    - **estimate** — the fixture's position from your camera pipeline
4. Record the error in each axis and the total distance, at all twelve poses.

**What to report**

Mean and maximum error, broken down two ways: **centre versus edge** of the board, and **low versus high** above it. Then account for what you see.

{: .highlight}
Expect it to be worse at the edges, and worse high up. Three reasons are worth separating: the board is viewed obliquely near the edges, depth is noisiest at grazing incidence, and all four tags sit in one region of the board — so a pose fitted to them is being *extrapolated* everywhere else. Which of those dominates is something your numbers can tell you.

{: .sanity_check}
You can state your calibration's accuracy as a number with a region attached — not "a few millimetres" but "3 mm near the middle, 12 mm at the far corner, and worse above 200 mm."

---

# Part B — Forward kinematics

## Task 2.6  Forward kinematics

Given six joint angles, where is the end effector? On hardware the arm answers this itself, and the GUI readout comes from the vendor's solver. The simulator has no such thing — `get_ee_pose_sdk()` returns zeros in sim, and the readout is driven entirely by your code.

{: .highlight}
**Run this task in simulation.** The end-effector readout in `--sim` *is* your forward kinematics. It is blank until you write it, correct when you get it right, and wrong in ways you can see — which makes the simulator a better test harness for this than the real arm, where the vendor's answer would mask your mistakes.

**Instructions**

1. In `src/kinematics.py`, implement **one** of the two methods:
    - **DH** — fill in `DH_STD`, then `get_transform_from_dh()` and `FK_dh()`
    - **PoX** — fill in `M` and `S_list`, then `to_s_matrix()` and `FK_pox()`
2. Implement `get_pose_from_T()` to extract `[x, y, z, roll, pitch, yaw]` from the transform.
3. Start the simulator and the control station with `--sim`, and watch the end-effector readout as you jog the joints.

{: .note}
`SimArm.get_ee_pose()` currently calls `FK_pox`, with the `FK_dh` line commented out just above it. If you implement the DH method, swap those two lines.

**Four ways to check your answer**

| Check | How |
| ----- | --- |
| Self-test | `python src/kinematics.py` — with all zeros, your DH transform should come back as the identity |
| The zero pose | All six joints at zero puts the flange at **(87, 0, 154.2) mm**. See [Hardware](/docs/armlab/hardware#zero-position) |
| Visual | `set_fk_display()` draws your computed frame in the MuJoCo viewer. If it sits on the arm's flange, your FK is right — if it floats off, it is not |
| Against the vendor | On the real arm, compare your `get_ee_pose()` against `get_ee_pose_sdk()` across a spread of configurations |

**Hints**

- The codebase is **radians and millimetres**. See [Units and conventions](/docs/armlab/software#units-and-conventions).
- Orientation is roll-pitch-yaw about **fixed** axes, matching the SDK — not ZYZ Euler.
- FK gives you the **flange**. The point that grasps a block is 85 mm further along the tool axis; you will need that in Checkpoint 3.
- The visual check is the fastest way to find a sign error. A joint turning the wrong way puts the frame somewhere obviously wrong long before the numbers make it clear.

{: .sanity_check}
In simulation, jogging any joint moves the end-effector readout the way you would expect, and your displayed frame stays locked to the flange through the whole range of motion.

---

## Deliverables

Submit the following on Canvas.

{: .submission}
**1)** Your extrinsic matrix from Task 2.2, with a statement of which direction it maps, and a comparison against the hand-measured one from Checkpoint 1. <br>
**2)** A short video of the calibration: the GUI before calibrating, the camera re-aimed, then Calibrate pressed, showing the Workspace view square up again. <br>
**3)** A figure of your grid projection (2.5a) and your board-plane error map (2.5b). <br>
**4)** Your twelve-pose fixture table (2.5c): truth, estimate and error per axis, with mean and maximum split by region and by height. <br>
**5)** Your DH table, or your `M` matrix and screw axes, with a schematic of the arm showing the frames or axes you used. <br>
**6)** A short video of the simulator with your FK running: jogging the joints, with the end-effector readout and your displayed frame tracking the arm.

{: .required_for_report}
From this checkpoint, carry the following into your final report: <br>
**1)** The extrinsic matrix and how you obtained it, compared against the hand-measured result. <br>
**2)** The equations that take a pixel and a depth reading to a world coordinate. <br>
**3)** Your homography matrix, and which points you used to compute it. <br>
**4)** Your calibration accuracy: the grid projection figure, the board-plane error map, and the twelve-pose table — with your account of where the error comes from and how it varies across the workspace. <br>
**5)** Your DH table or screw axes with the frame schematic. <br>
**6)** How you verified your forward kinematics, and the error you estimate it has.
