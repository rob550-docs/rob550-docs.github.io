---
layout: default
title: Checkpoint 2
nav_order: 2
parent: Checkpoints
grand_parent: Armlab
last_modified_at: 2026-09-03 12:00:00 -0400
---

This checkpoint has two halves. First, forward kinematics: given six joint angles, where is the end effector? The real arm answers that for itself, so you will build and test your own in the simulator, where nothing answers it but your code.

Then the camera. In Checkpoint 1 you measured its position with a tape measure and found out how rough that answer was. Here you replace it: the AprilTags on the board have known positions, so the camera can work out where it is by itself. You will build that calibration, turn it into the pixel-to-world pipeline the rest of the lab depends on, and then measure how good it is, not just on the board surface, but through the workspace in three dimensions.

### Contents
* TOC
{:toc}

## Before you start

- Checkpoint 1 is done: you can drive the arm, you have calibrated intrinsics, a hand-measured extrinsic matrix, and measurements of your board.
- You know where your AprilTags sit relative to the robot base frame. You need those for Part B. If you have not measured them yet, do it before you get there.

---

# Part A: Forward kinematics

## Task 2.1  Forward kinematics

Given six joint angles, where is the end effector? You will answer it twice, by two different formulations: the Denavit-Hartenberg convention here, and the product of exponentials in Task 2.3. On hardware the arm answers it for itself, and the GUI readout comes from the vendor's solver. The simulator has no such thing: `get_ee_pose_sdk()` returns zeros in sim, and the readout is driven entirely by your code.

{: .highlight}
**Run this task in simulation.** The end-effector readout in `--sim` *is* your forward kinematics. It is blank until you write it, correct when you get it right, and wrong in ways you can see, which makes the simulator a better test harness for this than the real arm, where the vendor's answer would mask your mistakes.

**Instructions**

1. In `src/kinematics.py`, fill in `DH_STD` from the arm's geometry: one row per joint, `[theta_offset, d, alpha, a]`.
2. Implement `get_transform_from_dh()` for a single row, and `FK_dh()` to chain them.
3. Implement `get_pose_from_T()` to extract `[x, y, z, roll, pitch, yaw]` from the transform.
4. Start the simulator and the control station with `--sim`, and watch the end-effector readout as you jog the joints.

{: .note}
`SimArm.get_ee_pose()` ships calling `FK_pox`, with the `FK_dh` line commented out just above it. Swap those two lines to drive the readout from your DH implementation. You will swap them back in Task 2.3.

**Four ways to check your answer**

| Check | How |
| ----- | --- |
| Self-test | `python src/kinematics.py`. With all zeros, your DH transform should come back as the identity |
| The zero pose | All six joints at zero puts the flange at **(87, 0, 154.2) mm**. See [Hardware](/docs/armlab/hardware#zero-position) |
| Visual | `set_fk_display()` draws your computed frame in the MuJoCo viewer. If it sits on the arm's flange, your FK is right. If it floats off, it is not |
| Against the vendor | The arm has its own FK. Comparing against it properly is Task 2.2 |

**Hints**

- The codebase is **radians and millimeters**. See [Units and conventions](/docs/armlab/software#units-and-conventions).
- Orientation is roll-pitch-yaw about **fixed** axes, matching the SDK, not ZYZ Euler.
- FK gives you the **flange**. The point that grasps a block is 85 mm further along the tool axis; you will need that in Checkpoint 3.
- The visual check is the fastest way to find a sign error. A joint turning the wrong way puts the frame somewhere obviously wrong long before the numbers make it clear.

{: .sanity_check}
In simulation, jogging any joint moves the end-effector readout the way you would expect, and your displayed frame stays locked to the flange through the whole range of motion.

## Task 2.2  Check your FK against the vendor's

The viewer overlay tells you the FK is roughly right. This task tells you *how* right, and when it is wrong, which parameter is at fault.

The arm carries its own forward kinematics, and `get_forward_kinematics()` takes joint angles as an **argument** rather than reading the arm's current position. So the arm never moves for any of this; it only needs to be connected. Both sides are already in the same units, since the arm object is built with `is_radian=True`.

{: .note}
This one needs the real arm. In `--sim` there is no vendor solver to compare against: `get_ee_pose_sdk()` returns zeros.

**Instructions**

1. Write a script that repeats the following a few hundred times:
    - draw a random joint vector `q`, each joint uniform within its row of `JOINT_LIMITS`
    - ask the vendor: `code, pose_sdk = arm.xarm.get_forward_kinematics(q)`, skipping any sample where `code != 0`
    - ask your own: `get_pose_from_T(FK_dh(DH_STD, q, 6))`
    - record both
2. For each sample, compute two errors:
    - **position**: the Euclidean distance between the two `xyz`, in mm
    - **orientation**: see below
3. Report the mean, median, maximum and 95th percentile of each.
4. Plot a histogram of the position error, and scatter the error against each joint angle in turn, looking for structure.

**Measuring orientation error**

{: .warning}
Do not subtract the two roll-pitch-yaw triples. Euler angles wrap at ±180° and the same rotation has more than one representation, so a perfectly correct FK can appear to be 179° out. Compare the rotations themselves.

Build the rotation matrix from each pose, then:

```
R_err = R_mine.T @ R_sdk
angle = arccos((trace(R_err) - 1) / 2)
```

That gives one number: the angle of the shortest rotation taking one frame to the other, and zero when they agree.

**Sample the whole joint space**

Test only near the home pose and you will pass with a table that is wrong somewhere else. Checkpoint 3 drives this arm into the far corners of its range, and that is where an untested error will surface.

**Isolating a bad table from bad chaining**

Run the same sweep twice, changing one argument:

```python
FK_dh(DH_STD, q, 6)            # your table
FK_dh(arm.dh_params, q, 6)     # the firmware's table, your code
```

On hardware `arm.dh_params` is read from the arm itself, so the second call runs *your* chaining code on the *vendor's* parameters. Between them:

| Result | What it means |
| ------ | ------------- |
| The two runs agree with each other | Your `DH_STD` matches the firmware's table |
| They disagree | Your chaining is right and `DH_STD` is wrong. The size of the gap tells you how wrong |
| Both are far from the vendor by the same amount | Expected. See the accuracy note below |

{: .note}
The firmware's table has **seven** rows where `DH_STD` has six. Working out why is worth a minute of your time.

**Reading the pattern**

When the numbers disagree, the shape of the disagreement points at the cause:

| What you see | What it usually means |
| ------------ | --------------------- |
| Error ≈ 0 everywhere | Correct |
| A constant offset in the same direction regardless of `q` | A wrong `d` or `a` in one row, or a tool offset set on the arm that your FK does not model |
| Zero at the zero pose, growing with one joint's angle | A wrong `alpha` or `theta` offset in that joint's row |
| Position right, orientation wrong | The wrist rows, or a frame convention at the end of the chain |
| Orientation error clustered near 90° or 180° | An axis convention flipped, not a numerical problem |
| Large and unpatterned | Transforms chained in the wrong order |

{: .highlight}
**Aim to agree with the vendor to within 10 mm.** You will not do better than a few millimeters, and that is not your fault: every arm carries a per-arm factory calibration in its controller, the firmware uses it for FK, and the published DH table does not include it. Measured across several of our arms, that accounts for 2 to 7 mm, varying with configuration. What you *should* see is a smooth residual with structure, not a wild one. If your error is tens of millimeters, or jumps, or grows without bound, that is a real bug sitting on top of the calibration floor. [Factory Calibration](/docs/armlab/factory-calibration) explains where those millimeters go and, optionally, how to remove them.

{: .sanity_check}
Your maximum position error against the vendor over several hundred random configurations is under 10 mm, and the error varies smoothly with configuration rather than jumping around.

## Task 2.3  Forward kinematics again, by product of exponentials

Same arm, same answer, different formulation. Where DH walks link by link through a chain of frames, the product of exponentials describes each joint as a **screw axis fixed in the base frame** at the home configuration, and composes the whole motion as a product of matrix exponentials. Deriving both is the point: they fail in different ways, and agreeing with each other is evidence neither alone can give you.

**Instructions**

1. In `src/kinematics.py`, fill in:
    - `M`: the 4 × 4 transform of the end effector at the **zero configuration**
    - `S_list`: six rows, one screw axis per joint, each `[w1, w2, w3, v1, v2, v3]` in the base frame
2. Implement `to_s_matrix()` to build the 4 × 4 `[S]` from a screw axis, and `FK_pox()` to chain the exponentials.
3. Swap `SimArm.get_ee_pose()` back to the `FK_pox` line and confirm the simulator behaves exactly as it did with DH.
4. Re-run your Task 2.2 sweep against the vendor, with `FK_pox` in place of `FK_dh`.

**Building the screw axes**

For a revolute joint, with the arm at its home configuration:

- `w` is the unit vector along the joint's rotation axis, expressed in the **base** frame
- `v = -w × p`, where `p` is **any** point lying on that axis, also in the base frame

Any point on the axis gives the same `v`, which is a useful thing to check yourself with. Pick two different points on the same axis and confirm you get the same answer.

{: .note}
Units are millimeters, as everywhere else in this file. `w` is dimensionless, `v` carries the length.

**Getting `M` right first**

`M` is the end-effector transform when every joint is at zero, so its translation is the zero pose from the [Hardware page](/docs/armlab/hardware#zero-position): **(87, 0, 154.2) mm**. The rotation is the flange's orientation in that configuration, which you derive from the same drawing.

{: .highlight}
If your DH implementation is working, `FK_dh(DH_STD, zeros, 6)` **is** `M`, which is what the home configuration means. Use it to confirm you have `M` right before you go near the screw axes. Note the consequence, though: an `M` obtained that way inherits any error in your DH table, so DH and PoX agreeing is no longer fully independent evidence. The comparison against the vendor is what stays independent.

**Two comparisons**

1. **Against the vendor**, exactly as in Task 2.2: same sweep, same two metrics, same 10 mm target and the same calibration floor underneath it.
2. **Against your own DH**, over the same random configurations. This one has no calibration floor, because both describe the same nominal arm, so it should agree to floating-point noise. It needs no vendor at all, and it is the check you would still have on a robot whose manufacturer gave you nothing.

The failure patterns differ from DH's in a useful way. A wrong `w` shows up as an error that grows with that joint's angle and vanishes at zero. A wrong `v`, usually from picking a point that is not actually on the axis, shows up as a position error that persists even when that joint sits at zero. A wrong `M` offsets everything uniformly, at every configuration.

{: .sanity_check}
`FK_pox` and `FK_dh` agree **with each other to floating-point noise** across several hundred random configurations, and both land within 10 mm of the vendor. The simulator readout is identical whichever one `SimArm` is wired to.

---

# Part B: Automatic camera calibration

{: .important}
`TAG_WORLD_POINTS` at the top of `src/camera.py` ships with placeholder positions. Replace them with the positions you measured. A calibration solved against the wrong world points will still return a matrix, and it will be wrong.

## Task 2.4  Draw the tag detections

Before trusting the detector, look at what it sees. `VideoThread` already runs the detector for you and leaves the results in `self.tag_detections`, refreshed every sixth frame.

**Instructions**

1. Implement `draw_tags_in_rgb_image()` in `src/camera.py`. For every detection, draw:
    - the tag's **ID**
    - a marker on its **center**
    - its **outline**, from the four corners
2. Select the **Tags** view in the GUI to see your work.

**Hints**

- Each detection carries `tag_id`, `center` and `corners`. The corners come back in a consistent order, which is what lets you draw an outline rather than a scatter of dots.
- Draw onto a copy of `self.video_frame`, and assign the result to `self.tag_image_frame`.
- The detector runs at about 5 Hz while the video runs at 30, so the overlay lags anything moving quickly through the frame. That is expected.

{: .sanity_check}
All four tags are found, the IDs you draw match the ones printed on the stickers, and each outline sits on the tag's black border rather than near it. Pass a hand over one tag and it should drop out of the overlay and come back. That confirms you are drawing live detections and not a fixed list.

## Task 2.5  Solve the extrinsics from the tags

Now compute the camera pose automatically, and put it behind the **Calibrate** button that is already wired to the `calibrate` state.

**Instructions**

1. Fill in `TAG_WORLD_POINTS` with your measured tag positions.
2. Implement `estimate_extrinsics_from_tags()`. Given tags detected in the image and their known world positions, solve for the transform between the two frames.
3. Store the result in `self.extrinsic_matrix` (world → camera) and cache the inverse in `self.extrinsic_matrix_inv`. Set `self.camera_calibrated`.
4. Return a helpful `(ok, message)`. That message is what appears in the GUI status bar.
5. Compare the matrix against the one you measured by hand in Checkpoint 1.

**Hints**

- `cv2.solvePnP` takes your known world points, the matching image points, and `self.intrinsic_matrix`, and returns a rotation vector and translation. `cv2.Rodrigues` turns the rotation vector into a matrix.
- The detector is currently called without pose estimation, so detections give you corners and centers but not tag poses. Either work from the centers, or use all four corners of every tag, which gives you sixteen points instead of four and a much better-conditioned solve. `TAG_SIZE_MM` is defined for exactly this.
- Sanity-check the answer before you trust it: the camera sits about 1 m above the board looking down, so the translation should say so.
- Be explicit with yourself about which direction your matrix maps. Storing world → camera when you meant camera → world produces results that look almost right, which is the worst kind of wrong.

{: .sanity_check}
Pressing Calibrate reports success, and the recovered camera position is within a few centimeters of where you measured it in Checkpoint 1.

## Task 2.6  From pixels to the world

The calibration is only useful once it can answer the question the rest of the lab asks constantly: *where is this pixel on the board?*

**Instructions**

Implement the three functions that make up the chain:

1. `depth_to_camera_point(x, y, depth_raw)`: a pixel plus a raw depth reading, to a 3D point in the **camera** frame.
2. `camera_to_world(camera_point)`: camera frame to **world** frame.
3. `image_to_world(x, y)`: the whole chain, using live depth.

**Hints**

- Raw depth units are not millimeters. Multiply by `self.depth_scale_mm`.
- Depth is already aligned to color, so `(u, v)` means the same point in both images. That alignment is why this works at all.
- Deprojection is the intrinsic matrix run backwards: `X = (u − cx)·Z / fx`, `Y = (v − cy)·Z / fy`.
- Depth is noisy on dark, shiny or steeply angled surfaces, and returns zero where it has no reading. Decide what your code does with a zero.

{: .sanity_check}
Hover over a grid intersection you know the coordinates of. The world readout under the video should land within a few millimeters, and read `(0, 0)` at the base of the arm.

## Task 2.7  The top-down workspace view

The camera looks at the board from an angle, so the board appears as a trapezoid. A homography maps it back to a rectangle, which makes everything downstream easier to reason about.

**Instructions**

1. Implement `_update_workspace_transform()`: build the homography from four source points in the image to four destination points forming a rectangle. Store both it and its inverse.
2. Implement `update_workspace_frame()` to warp the video into `self.workspace_frame`.
3. Implement `workspace_pixel_to_image()` so that clicking in the rectified view still maps back to the right pixel in the original image.
4. Select the **Workspace** view to check it.

**Hints**

- Source points can be the tag centers (you have those pixel coordinates already) or the board corners (you know their world coordinates and now have the means to project them into the image).
- Choose destination points that keep the board's aspect ratio. Stretching it will not break the math but will make everything harder to look at.
- Step 3 is the one people forget. The mouse readout and Click Pick & Place both go through it, and if it is missing, clicks in the Workspace view land somewhere else entirely.

{: .sanity_check}
The board fills the Workspace view square and level, the grid lines run parallel to the edges of the frame, and hovering still reports correct world coordinates.

## Task 2.8  Measure how good your calibration is

A calibration you have not measured is a guess. This task is about producing evidence, and finding where it fails, because it will fail somewhere.

### 2.8a  Project the grid

Write a function that takes the known world coordinates of the board's grid intersections, projects them into the image through your intrinsics and extrinsics, and draws them over the video.

Where the projected points sit on the real grid lines, your calibration is good. Where they drift off, it is not. Look especially at the edges and corners of the board.

### 2.8b  Check the board plane

The board surface is `z = 0` everywhere. So sample a grid of pixels across the empty board, run each through `image_to_world`, and record the world `z` you get back. Every one of them should be zero.

Turn the results into a picture: a heat map, or contours over the board. This costs you no measuring at all and shows exactly how the error is distributed across the workspace.

### 2.8c  Check it in three dimensions

The board plane is only one slice of the workspace. To test the rest, you need targets whose true positions you know at heights above the board. There are two ways to get them. Use the first if your station has a calibration fixture; otherwise use the second. Either way you end up with **four board locations × three heights**, twelve points.

**Option 1: the calibration fixture**

The fixture is a 3D-printed AprilTagged object the gripper grips in a known, repeatable way, so the tag's pose relative to the end-effector frame is known to a fraction of a millimeter. That makes the arm your ruler, and it is a far better one than a tape measure.

1. Grip the fixture.
2. Drive the arm to four board locations at roughly `z` = 50, 150 and 250 mm, using the jog controls or waypoints from Checkpoint 1.
3. At each pose, record two things:
    - **truth**: the arm's reported end-effector pose, plus the known fixture offset
    - **estimate**: the fixture's tag position from your camera pipeline
4. Record the error in each axis and the total distance, at all twelve poses.

**Option 2: block stacks**

You measured your blocks in Checkpoint 1, and the board's grid gives you known positions. That is enough to build targets at known heights by hand.

1. Choose four grid intersections: two near the middle of the board, two near its edges.
2. At each one, in turn, place a stack of one block, then two, then three.
3. For each stack, record two things:
    - **truth**: the grid coordinates, and `z` = the number of blocks × the block height you measured
    - **estimate**: hover the mouse over the center of the top face and read the world coordinate from the readout under the video
4. Record the error in each axis and the total distance, at all twelve points.

{: .note}
Option 2 is less precise: your placement of the stack and your click on its center each contribute a millimeter or two, on top of whatever the calibration gets wrong. Say so when you report the numbers, and do not read a 2 mm error as a calibration error when it could be your click.

**What to report**

Mean and maximum error, broken down two ways: **center versus edge** of the board, and **low versus high** above it. Then account for what you see.

{: .highlight}
Expect it to be worse at the edges, and worse high up. Three reasons are worth separating: the board is viewed obliquely near the edges, depth is noisiest at grazing incidence, and all four tags sit in one region of the board, so a pose fitted to them is being *extrapolated* everywhere else. Which of those dominates is something your numbers can tell you.

{: .sanity_check}
You can state your calibration's accuracy as a number with a region attached, not "a few millimeters" but "3 mm near the middle, 12 mm at the far corner, and worse above 200 mm."

---

## Deliverables

Submit the following on Canvas.

{: .submission}
**1)** Your DH table **and** your `M` matrix and screw axes, with a schematic of the arm showing the DH frames and the screw axes you used. <br>
**2)** Your Task 2.2 verification: the error statistics for position and orientation, a histogram of the position error, and a sentence on what the pattern told you and what you changed. <br>
**3)** Your Task 2.3 results: the PoX sweep against the vendor, and the agreement between your PoX and DH implementations over the same configurations. <br>
**4)** Your extrinsic matrix from Task 2.5, with a statement of which direction it maps, and a comparison against the hand-measured one from Checkpoint 1. <br>
**5)** A short video of the calibration: the GUI before calibrating, then the camera re-aimed on its bracket, then Calibrate pressed, showing the Workspace view square up again. <br>
**6)** A figure of your grid projection (2.8a) and your board-plane error map (2.8b). <br>
**7)** Your twelve-point 3D table (2.8c), stating which option you used: truth, estimate and error per axis, with mean and maximum split by region and by height.

{: .required_for_report}
From this checkpoint, carry the following into your final report: <br>
**1)** Your DH table and your screw axes, with the frame schematic for each. <br>
**2)** How you verified both forward kinematics implementations against the arm's own solver and against each other, the error metrics you used, and the agreement you achieved. <br>
**3)** The extrinsic matrix and how you obtained it, compared against the hand-measured result. <br>
**4)** The equations that take a pixel and a depth reading to a world coordinate. <br>
**5)** Your homography matrix, and which points you used to compute it. <br>
**6)** Your calibration accuracy: the grid projection figure, the board-plane error map, and the twelve-point 3D table, with your account of where the error comes from and how it varies across the workspace.
