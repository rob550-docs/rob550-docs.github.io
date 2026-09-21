---
layout: default
title: Checkpoint 3
nav_order: 3
parent: Checkpoints
grand_parent: Armlab
last_modified_at: 2026-09-21 12:00:00 -0400
---

Checkpoint 2 gave you two things: a forward kinematics you trust, and a camera that can turn a pixel into a point on the board. This checkpoint closes the loop between them. First inverse kinematics, so that a point in the world becomes a set of joint angles. Then a block detector, so that the camera finds the blocks rather than you clicking on them. Then the two together: click a block, and the arm picks it up.

### Contents
* TOC
{:toc}

## Before you start

- Checkpoint 2 is done. In particular, both FK implementations agree with the vendor's solver to floating-point noise, and the Workspace view and the pixel-to-world chain work.
- You know your block sizes. The two used in this lab are about **38 mm** and **25 mm** on a side; use the numbers you measured.
- You have the *From Pixels to the Workspace* lecture to hand. Part B follows it step for step.

---

# Part A: Inverse kinematics

Forward kinematics answered "given the joints, where is the flange?" Inverse kinematics is the question the rest of the lab actually asks: given where I want the flange, what are the joints? You will write it twice, as you did FK. A closed-form solution that exploits the arm's geometry, and a numerical one that only needs your FK. They fail differently, and each one checks the other.

{: .highlight}
**Work in simulation.** Both solvers can be developed and tested entirely in `--sim`, where the IK panel already shows your geometric solution and your numerical solution side by side, flags joints outside their limits, and draws a ghost of the solution in the viewer before you commit to it. Go to the hardware only for the vendor comparison in Task 3.3.

**What the solvers receive.** Both take a full pose, `[x, y, z, roll, pitch, yaw]` in mm and radians, and that pose is the **flange**. The grasp point is 85 mm further along the tool axis, so to put the gripper somewhere you first move the target 85 mm back along the direction the tool will point. For a top-down grasp the tool points straight down, so the flange sits 85 mm above the grasp point.

## Task 3.1  Geometric inverse kinematics

The Lite 6 has a **spherical wrist**: the axes of joints 4, 5 and 6 meet at a single point. That is what makes a closed-form solution possible, because it lets the problem split in two.

**Instructions**

Implement `IK_geometric(dh_params, pose)` in `src/kinematics.py`.

1. **Find the wrist centre.** The flange sits a fixed distance along the final joint axis from where the three wrist axes meet. Take the target position, and step back along the target's tool axis by that distance. You now have a point whose position depends only on joints 1, 2 and 3.
2. **Solve joint 1** from where the wrist centre sits when viewed from above. On this arm there is no lateral shoulder offset, so this is a single `atan2`.
3. **Solve joints 2 and 3** as a two-link planar problem in the vertical plane through joint 1. The upper arm is one link. The forearm is the other, but it has an elbow offset: the two DH lengths between joints 3 and 4 form a fixed right angle, so treat their hypotenuse as the link and fold the fixed angle into joint 3. The law of cosines gives you joint 3; joint 2 follows from the remaining triangle.
4. **Solve joints 4, 5 and 6.** Compute the rotation your first three joints produce, then the rotation still needed to reach the target orientation. Extract the three wrist angles from that remaining rotation, respecting the DH frame conventions your table encodes.
5. **Check every joint against `JOINT_LIMITS`.** If a solution is outside them, it is not a solution. Return `None` when nothing reachable exists.

**Multiple solutions, and which to return**

A spherical-wrist arm has up to eight solutions for one pose: elbow up or down, wrist flipped or not, and joint 1 turned round or not. Your solver has to pick one, and it has to pick consistently, or the arm will lurch between configurations on consecutive moves.

- The joint limits do some of the choosing for you. Joint 3's range is −3.5° to 300°, which admits only one of the two elbow branches. Work out which.
- For the rest, match the branch that `Q_DEFAULT` sits in. The home pose is the configuration the arm is expected to work from, and the numerical solver is seeded there, so agreeing with it keeps the two solvers comparable.

**Hints**

- Every length you need is already in your `DH_STD` from Checkpoint 2. Do not type numbers into the solver; read them from the table, so that a corrected table corrects the solver.
- The law of cosines tells you about reachability before it tells you an angle: if the cosine it produces is outside `[−1, 1]`, the wrist centre is beyond the arm's reach. That is your first `None`.
- **Singularities.** When joint 5 is zero, joints 4 and 6 spin about the same axis and only their sum is determined. Detect the case and choose one, for instance keep joint 4 at zero, rather than dividing by something that is nearly zero. When the wrist centre lies on the joint 1 axis, joint 1 is undefined in the same way.
- The IK panel's ghost preview is the fastest way to see a wrong branch. The ghost lands on the right point with its elbow the wrong way, and the mistake is obvious at a glance.

{: .sanity_check}
Enter the flange pose that your FK reports at `Q_DEFAULT` and the solver returns `Q_DEFAULT` back. Enter a pose well beyond 440 mm from the base and it returns `None` rather than an angle.

## Task 3.2  Numerical inverse kinematics

A numerical solver does not know anything about the arm's geometry. It only knows your FK, and it searches for the joint angles that make FK produce the target. That makes it general, and it makes it a completely independent check on the geometric solution.

**Instructions**

Implement `IK_numerical(dh_params, pose, q0, joint_limits, w_rot)` as **bounded Gauss-Newton least squares**: iteratively minimise the error between `FK(q)` and the target pose, keeping every joint inside its limits.

1. **Define the residual.** Six numbers: the position error in mm, and the orientation error as a rotation vector, scaled by `w_rot` so that both blocks are in comparable units. You already know how to measure orientation error properly from Checkpoint 2; a rotation vector is the same idea, keeping the direction as well as the angle.
2. **Compute the Jacobian** of that residual with respect to the six joints. Finite differences on your FK is entirely adequate here: six evaluations of a cheap function.
3. **Step.** Solve the linear least-squares problem for the update, apply it, and clip each joint back inside its limits.
4. **Iterate** until the residual is below a tolerance, or you have taken too many steps. Return `None` if you did not converge.
5. **Seed from `Q_DEFAULT`** when `q0` is not given, so that the same target always produces the same answer.

**Hints**

- Plain Gauss-Newton can overshoot when the Jacobian is nearly singular, which is exactly what happens near the wrist singularity. Adding a small damping term to the normal equations (the Levenberg-Marquardt idea) costs one line and removes most of the bad behaviour.
- `w_rot` is in mm per radian. At 200 it says a radian of orientation error is as bad as 200 mm of position error, which only matters when the pose cannot be reached exactly and the solver has to trade one against the other.
- **Decide what "unreachable" means for this solver.** Unlike the geometric one, it will happily converge to the nearest reachable pose and report it. If the final residual is larger than your tolerance, that is a failure and you should return `None`, not the best it could do.
- Compare against your geometric solver on the same pose. They will not return the same joint angles if they land on different branches, but `FK` of each must give the same pose.

{: .sanity_check}
For a reachable pose, the numerical solver converges in a handful of iterations and `FK` of its answer matches the target to well under a millimetre. For one just out of reach it returns `None` instead of a nearby pose.

## Task 3.3  Verify both solvers, and map what you can reach

You verified FK by comparison. IK gets the same treatment, plus something FK could not give you: a picture of where on the board a block can actually be picked up.

**Round trip**

1. Draw a few hundred random joint vectors inside `JOINT_LIMITS`, as you did in Checkpoint 2.
2. For each, compute the pose with your FK, then send that pose to each IK solver, then run FK on the result.
3. The pose has to come back. Compare poses, not joint angles: a different branch is a different set of angles reaching the same place, and that is not an error.
4. Report position and orientation error, mean and maximum, for each solver.

**Against the vendor**

On the real arm, `arm.xarm.get_inverse_kinematics(pose)` returns the vendor's answer. Run the same sweep and compare, again through FK. This one needs the hardware, but the arm never moves.

**The reachability map**

For the competition you need to know, before you try, whether a block at a given spot can be picked up from above. Build that map now.

1. Choose the flange pose for a top-down grasp of a 38 mm block on the table: tool pointing straight down, flange 85 mm above the block's centre height.
2. Sweep `x` and `y` across the whole board on a 10 mm grid. At each point, ask your geometric solver for that pose with the gripper aligned to the board's grid, and again with it turned 90°.
3. Mark the point reachable if a solution exists inside the joint limits.
4. Plot the result over an outline of the board: reachable, unreachable, and the board's edges and the arm's base for reference.

Repeat for the 25 mm block if the two maps differ noticeably; they may not.

{: .highlight}
Expect a shape that is nothing like a circle. The 440 mm reach sets the outer edge, but joint 5's ±124° limit and the elbow range carve out a region close to the base where the wrist cannot point straight down, and the far corners of the board fall off entirely. This map is going into your report, and it should shape where you put things during the competition.

{: .sanity_check}
Both solvers round-trip to floating-point noise on every reachable configuration, both agree with the vendor through FK, and you have a plotted map of the pickable region of the board.

---

# Part B: Block detection

Everything so far has needed you to click. Now the camera does the finding. The approach follows the lecture: get to a top-down view of the board where the table is flat, threshold height to find the blocks, and only then look at colour.

{: .note}
This part needs the real camera. There is no image in the simulator. Save a few RGB and depth pairs of the board with blocks on it early, and develop against those, so that the whole team is not waiting on one station. Keep depth as 16-bit PNG in raw units, exactly as the camera delivers it.

## Task 3.4  Find the blocks

Write the detector as a set of functions in `src/camera.py`, run from `VideoThread` in the same way `update_workspace_frame()` is. There is no stub. Each detection should end up as a record with the block's **world position**, **colour**, **size** and **orientation**, and the results should be drawn on a view in the GUI so you can see what the detector sees.

**1. Depth in the top-down view**

You already warp the RGB image into the Workspace view. Warp the depth image with the same homography. Use nearest-neighbour interpolation for depth: bilinear would average a valid reading with an invalid zero, or across a block edge, and invent depths that exist nowhere.

**2. Flatten the table**

The camera looks down at roughly 19°, so the empty table alone spans about 150 mm of depth from its far edge to its near edge. A 38 mm block cannot be picked out of that ramp with a single threshold.

The table is a plane, and in the top-down view the depth of a plane is affine in pixel position. So: sample the depth at the four corners of the board (a median over a small window, so one bad pixel cannot bias it), fit `z = a·u + b·v + c` through them, subtract that plane from every pixel, and re-centre on the mean corner depth so the numbers stay in millimetres. Height above the table is then simply that mean depth minus the flattened depth.

{: .warning}
The corners must be on the table. A sample that lands on a rail, a tag, or the arm puts a large error into the plane and every height after it.

**3. Correct the depth bias, if you need to**

The L515's reported range drifts with elevation angle: on the lab cameras the table reads about 1 mm too far at the far edge and 13 mm too far at the near edge, with a slight side-to-side tilt. Your extrinsics tell you the true range to the table along every pixel's ray. Fit the difference over the whole table with a smooth surface, rejecting blocks and the arm as outliers, and subtract it once.

You may not need this for single blocks. You will need it to count stacks reliably, because the second and third levels sit at multiples of the block height and a 13 mm bias eats most of the margin.

**4. Threshold height, then find contours**

With the table flat, one height band finds every block: something like 15 mm to 60 mm keeps block tops and rejects the table, the tags and noise. Clean the mask with a morphological open, then `cv2.findContours` with external retrieval.

For stacks, use bands at multiples of the block height, and count the levels a contour reaches.

**5. Size and orientation from the contour**

`cv2.minAreaRect` gives a rotated rectangle: its centre, its side lengths and its angle. In the top-down view the pixel scale is uniform, and you know it from the tag spacing you used to build the homography, so side lengths convert straight to millimetres. That separates 38 mm blocks from 25 mm ones. The angle is the block's orientation, and since blocks are square it only matters modulo 90°.

Filter contours by area before you trust them. Anything much smaller than a 25 mm block is noise; anything much larger is two blocks touching, or the arm.

**6. Colour from the RGB image**

Colour comes last, and only from pixels you already know belong to a block. Erode the contour mask a little so the sample does not include the block's edges, then take the median colour inside it. Classify in HSV by hue. Red wraps around zero, so it needs two ranges. Tune the ranges under the lab's actual lighting, on all six colours, and write the numbers down, because they are part of your method.

{: .note}
Median, not mean. A mean is pulled by specular highlights and the odd edge pixel; a median is not.

**7. Position in the world**

The rectangle's centre is a pixel in the top-down view. Send it back through the inverse homography to the original image, read the depth there, and run it through `image_to_world`. Use the **original** depth for this, not the flattened one: flattening was for thresholding, and it changed the numbers. Because the homography is exact on the table plane, a block's `x` and `y` from the top-down view and from the original view agree to within a millimetre.

**Hints**

- Develop on saved image pairs first, in a standalone script, with the intermediate images displayed. Once it works, move the functions into `camera.py`. Debugging inside the live GUI is slow.
- Draw every intermediate: the warped depth, the flattened depth, the mask, the contours, the rectangles. Most detector bugs are visible in exactly one of those pictures.
- Shadows and the tags themselves are the usual false positives on the colour side, which is why this pipeline segments on height first and looks at colour second.
- Blocks touching each other merge into one contour. You do not need to solve that for this checkpoint, but notice when it happens, because the competition will make you care.

{: .sanity_check}
With several blocks of different colours and both sizes on the board, the view shows each one outlined, with its colour, size and orientation labelled and its world coordinates printed, and none of the tags, the arm, or the empty table is labelled as a block.

## Task 3.5  Measure the detector

A detector you have not measured is a demo. Produce numbers.

1. **Position.** Place blocks at grid intersections you know, across the whole board, and compare detected against true position. Report the error and how it varies with distance from the centre.
2. **Colour.** Show the detector every colour, several times each, at several places on the board. Build a confusion matrix: which colours it gets right, and which it confuses with which.
3. **Size and orientation.** Place a block at known angles and confirm the reported orientation; place both sizes and confirm the classification.
4. **Detection rate.** How often is a block present but not found, and where on the board does that happen?

Turn the position result into a plot over the board, in the same style as your calibration error map from Checkpoint 2, so that the two can be compared.

{: .sanity_check}
You can state the detector's accuracy as numbers with regions attached, and you know which colour pair it confuses most and under what lighting.

---

# Part C: Click to grab, click to place

## Task 3.6  Click to grab, click to place

Now put the pieces together. Everything this needs already exists: the Click Pick & Place toggle drives the `pick_place` state, clicks arrive in `camera.last_click` and `camera.new_click`, and `image_to_world`, your IK, and the gripper are all yours.

**Instructions**

Implement `pick_place()` in `src/state_machine.py`.

1. Wait for a click. Convert it to a world point with `image_to_world`.
2. Compute the grasp pose: tool pointing down at that point, at the height of the block's centre. Move the target 85 mm up to get the flange pose.
3. **Approach from above.** Solve IK for the flange pose plus a clear margin above it, move there, then descend to the grasp pose. Never move laterally at grasp height.
4. Close the gripper, and give it time to finish.
5. Lift back to the approach height.
6. Wait for a second click. Repeat the approach for the place point, open the gripper, wait, and lift clear.
7. Handle failure. If IK returns `None` for either point, say so in the status message and stay in the state waiting for a better click, rather than moving at all.

**Hints**

- The gripper hangs 100 mm below the flange, so your approach height has to keep the flange more than 100 mm above anything the fingers could hit, including a stack the block is going onto.
- Slow down for this. The speed slider scales every command, and a wrong sign in your IK at full speed drives into the board.
- Use `wait=True` on every move so the sequence runs in order, and remember the gripper call returns before the jaws have finished travelling.
- Test it on the table plane first. Once a single block works, try placing onto a second block, which is the first time the approach height really matters.

**Stretch goal: aligned grasp**

Rather than a bare point, click a *detected block*, and use its orientation from Task 3.4 to set the gripper's yaw so the jaws close square across it. This is the difference between a grasp that works on a block sitting on the grid and one that works on a block sitting however it was dropped.

{: .sanity_check}
Click a block, and the arm approaches from above, picks it up, and waits. Click elsewhere, and it places the block there and returns. Click somewhere unreachable, and it tells you so without moving.

---

## Deliverables

Submit the following on Canvas.

{: .submission}
**1)** Your geometric IK derivation: the wrist-centre decoupling, the equations for each joint, and a schematic of the arm marking the wrist centre and the two-link triangle. <br>
**2)** Your verification from Task 3.3: round-trip error for both solvers, agreement with the vendor, and the branch policy you chose. <br>
**3)** The reachability map from Task 3.3. <br>
**4)** A short video of block detection with several colours and both sizes on the board, showing each block labelled with colour, size, orientation and world position. <br>
**5)** Your detector measurements from Task 3.5: the position error plot and the colour confusion matrix. <br>
**6)** A short video of click-to-grab and click-to-place, showing both the control station and the real arm.

{: .required_for_report}
From this checkpoint, carry the following into your final report: <br>
**1)** The inverse kinematics equations and the schematic, with the branch policy and how you handle singularities and unreachable poses. <br>
**2)** How you verified both solvers against each other and against the vendor, and the errors you measured. <br>
**3)** The plot of the region of the board from which a block can be picked up with a top-down grasp. <br>
**4)** Your block detection method, the strategies you used to limit false positives, and the evidence for its accuracy, including the position error plot. <br>
**5)** The motion plan you execute for a pick and a place, and how it fails safely.
