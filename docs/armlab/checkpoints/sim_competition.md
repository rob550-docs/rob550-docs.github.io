---
layout: default
title: Sim Competition
nav_order: 5
parent: Checkpoints
grand_parent: Armlab
last_modified_at: 2026-10-13 12:00:00 -0400
---

The MuJoCo simulation now has the Arm Lab gripper, the board and the twelve blocks, so you can write and test pick-and-place and competition code before your turn on the real arm. Block poses come straight from the sim, so no camera work is needed here, and the code you write runs unchanged on the real arm.

<video controls preload="metadata" playsinline
       poster="/assets/images/armlab/checkpoints/sim_competition_poster.jpg"
       style="max-width:800px; width:100%; border-radius:4px;">
  <source src="/assets/videos/sim_competition.mp4" type="video/mp4">
  Your browser cannot play this video. <a href="/assets/videos/sim_competition.mp4">Download it instead.</a>
</video>

Event 0 running in the simulator: the arm picks a block from in front of itself and sets it down in one of the slots behind. The clip has no sound.

{: .important}
**This is optional and carries no points.** It exists so that your competition code is written and debugged before you are standing at a station with the clock running. Everything on this page is self contained: you do not need anything from the [Competition](/docs/armlab/checkpoints/competition) page to work through it.

### Contents
* TOC
{:toc}

## Get the update

```bash
git pull upstream main
```

Use plain `git pull` if you cloned the course repository directly. Nothing has to be reinstalled, since `mujoco-sim` runs from your checkout.

{: .warning}
This update also changes four files you may have edited: `src/lite6arm.py`, `src/control_station.py`, `src/state_machine.py` and `src/ui/layout.py`. If git reports a conflict in one of them, keep **both** your lines and the update's lines. In `lite6arm.py`, take the update's `set_joint_angles`, which now has a `wait=` argument and returns `True` or `False`, and keep your own `XARM_IP`. Then `git add` the file and `git commit`.

## Run it

Two terminals, as usual. Terminal A runs the simulator:

```bash
mujoco-sim
```

Terminal B runs the control station against it:

```bash
cd src
python control_station.py --sim
```

A row of **Custom Buttons** along the bottom of the window runs events through the state machine:

<a class="image-link" href="/assets/images/armlab/checkpoints/sim_custom_buttons.png">
<img src="/assets/images/armlab/checkpoints/sim_custom_buttons.png" alt="The Custom Buttons row: Reset Board (Sim) and Event 0 are enabled, and Custom 3 through Custom 8 are greyed out" style="max-width:800px; width:100%;"/>
</a>

| Button | What it does |
| ------ | ------------ |
| Reset Board (Sim) | Drives the arm to the initial pose and lays the blocks out for Event 0 |
| Event 0 | One pick and place, `event_0` in `src/competition.py` |

The greyed buttons are yours to bind, as described in [Writing your own events](#writing-your-own-events): each one you add to `CUSTOM_BUTTONS` lights up the next slot. **STOP** stops the arm, which ends a running event; click **Initial Pose** to recover.

{: .highlight}
**These buttons are how you start a competition task on the real arm too.** The Custom Buttons row belongs to the control station, not to the simulator, so it is there in both modes and an event you bind here is the one you click at your station on competition day. The one exception is Reset Board (Sim), which lays blocks out in the simulator and reports `works in the sim only` if you click it on the real arm. Bind and debug your events here, and on the day you are pressing a button you have already tested.

## The board and the frame

The world frame is the arm base frame. The origin is the center of the arm mount, **x** points forward over the board, **y** to the arm's left, and **z** up, with `z = 0` at the board surface. Units are millimeters and radians, the same as everywhere else in the codebase.

The board is 100 by 60 cm, running from `x = -100` to `500` mm and `y = -500` to `500` mm, with a 50 mm grid. Note that it extends behind the arm as well as in front of it, which Event 0 uses.

## The gripper

Binary jaws, like the real one: `arm.open_gripper()`, `arm.close_gripper()` and `arm.stop_gripper()`. The jaws open to 50 mm and close on a block with a capped squeeze.

Two distances matter, and `src/competition.py` has both as constants:

| Constant | Value | What it is |
| -------- | ----- | ---------- |
| `GRASP_OFFSET_MM` | 85.0 | Flange to the middle of the finger pads, which is the grasp point |
| `FINGER_TIP_MM` | 96.4 | Flange to the finger tips |

The first is what turns a point you want to grasp into a flange pose. The second is what stops you driving the finger tips through the board or through the block below the one you are placing on.

## The blocks

Twelve cubes: 38 mm and 25 mm, one of each size in red, orange, yellow, green, blue and violet. Blocks that an event does not use wait beside the board.

`src/blocks.py` defines the one type that perception and competition code agree on:

```python
@dataclass
class Block:
    color: str      # red, orange, yellow, green, blue or violet
    size: float     # edge length (mm), LARGE_MM or SMALL_MM
    x: float        # centre of the block (mm)
    y: float
    z: float
    yaw: float      # rotation about world z (rad), wrapped to [-pi/4, pi/4)
```

It also has `block.is_large`, and `wrap_yaw()`, which folds an angle into `[-pi/4, pi/4)` because a cube looks the same every 90 degrees. There is no single correct yaw for a square block, only a class of equivalent ones, and wrapping is what lets you compare two of them.

## Seeing the blocks

`camera.detect_blocks()` returns a `list[Block]` giving the exact pose of every block on the board that a camera looking straight down would see, meaning no other block sits above its center. After you take a block off a stack, call it again to see the one underneath.

{: .note}
In the sim this comes from the MuJoCo ground truth, so it is exact and there is no calibration, no lighting and no color thresholds to get right. That is the point: it lets you work on the task rather than on perception. The perception you write for [Checkpoint 3](/docs/armlab/checkpoints/checkpoint3) returns the same list from the real camera.

## How the sim behaves like the real arm

The parts worth knowing before you write a sequence:

- `arm.set_joint_angles(q)` **queues** the move and returns at once. Moves run one after another.
- `arm.set_joint_angles(q, wait=True)` blocks until the move is done, and returns `False` if it failed or the arm was stopped.
- Gripper calls queue behind moves by default (`sync=True`), so `close_gripper()` straight after a queued move closes the jaws when the arm gets there. `sync=False` acts immediately. `wait=True` sleeps 0.8 s from the moment of the call, so it does **not** wait for queued moves. Gripper calls return nothing.
- **STOP** stops the arm, and motion calls keep returning `False` until you click Initial Pose.

{: .warning}
Check the return value of every motion call and give up early. After a STOP, a sequence that ignores return values will run all the way through doing nothing, close the gripper on air, and report success.

## Event 0

The warm-up. It is one pick and place, and there are no points.

- **Start.** All twelve blocks lie flat in front of the arm (`x > 0`), each at its own position and yaw. The layout is the same every time.
- **Each click** picks one random block in front of the arm and places it just behind the arm at `x = -25` mm, on the lowest of six slots at `y = -400, -300, -200, 200, 300` and `400` mm, taking the first one on a tie. So the first six clicks fill the slots in that order, and the next six stack a second layer on top.
- **Reset Board (Sim)** lays the blocks out again.

{: .note}
The slot positions are not arbitrary. Slots closer in than `|y| = 200` mm would bring the gripper near the arm base, and past `|y| = 400` mm is out of reach. Picking placements your arm can actually get to is a real part of this, and it is the same question the reachability map in Checkpoint 3 answers.

Read `event_0` in `src/competition.py` before you write anything of your own. It is short, it works, and it shows the shape every other event takes: detect, choose, pick, place, check each step, return home.

## Writing your own events

`src/competition.py` is yours. Alongside `event_0` it gives you helpers that do the fiddly parts:

| Helper | What it does |
| ------ | ------------ |
| `top_down_pose(x, y, z_grasp, yaw)` | Flange pose putting the grasp point at `(x, y, z_grasp)` with the jaws pointing down |
| `jaw_yaw(x, y, block_yaw)` | Gripper yaw that squares the jaws with a block of that yaw at that point |
| `grasp_height(block)` | Grasp point height for a block: its center, or higher if that is what keeps the finger tips clear |
| `move_to(arm, pose)` | Blocking joint move to a flange pose |
| `pick(arm, block, hover_height=None)` | Grasp a block from above and lift it |
| `place(arm, block, x, y, z_surface, yaw, hover_height=None)` | Set the held block down centered at `(x, y)` on a surface at height `z_surface` |

To put an event on a button, write it in `src/competition.py` as a function taking the arm and the camera:

```python
def my_event(arm, camera):
    ...
```

then add it to `CUSTOM_BUTTONS` in `src/control_station.py`:

```python
CUSTOM_BUTTONS = [
    ("Reset Board (Sim)", reset_board_event0),
    ("Event 0", competition.event_0),
    ("My event", competition.my_event),
]
```

A click has the state machine run it in its own thread. Buttons past the end of the list stay disabled.

{: .important}
The helpers call `arm.get_ik()`, which is **your** `IK_geometric` from `kinematics.py`, falling back to `IK_numerical` when it returns `None`. Everything here rests on the inverse kinematics you wrote in Checkpoint 3. If an event cannot reach a pose it should obviously be able to reach, suspect your solver before you suspect the helper.

## Taking it to the real arm

The same buttons run the same code on the real arm. The one thing that changes is where the blocks come from: implement `Camera.detect_blocks()` in `src/camera.py` as part of [Checkpoint 3](/docs/armlab/checkpoints/checkpoint3) so that it returns the same `list[Block]`, with block centers in the world frame in millimeters, the edge length, the color name and the yaw.

Get that right and your competition code does not know which one it is driving.
