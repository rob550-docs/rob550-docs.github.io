---
layout: default
title: Competition
nav_order: 6
parent: Checkpoints
grand_parent: Botlab
last_modified_at: 2024-11-17 16:37:48 -0500
---

**Update 11/5/24:** Change competition events as forklift is no longer used.

**Update 11/17/24:** Refine competition events to more utilize cones.

<a class="image-link" href="/assets/images/botlab/checkpoints/doge-meme.png">
<img src="/assets/images/botlab/checkpoints/doge-meme.png" alt=" " style="max-width:250px;"/>
</a>

## Contents
* TOC
{:toc}

## Event 1: Speed Run [300 points max]

You will complete two runs in total, one of which is judged for accuracy and the second which is judged for speed.

For the first run, make a circuit around a convex arena by driving the path given to you. Do this while running SLAM to make a map of the arena. Return to the starting pose. You are judged based on the final error in pose and your map quality. Save an image of your map and upload the image to Google Drive.

For the second run, drive quickly around the path two times. You are judged on the speed to complete the run. For this run, you will not be judged based on your final error, but your ending pose must be close enough to the start that you are still partially on the starting piece of paper. If your final position is beyond that then you will receive no points for the speed run section.

### Points

Error in pose after mapping:
- 100 points: < (3 cm, 3 cm, 15°)
- 75 points: < (5 cm, 5 cm, 15°)
- 50 points: < (10 cm, 10 cm, 30°)
- 25 points: < (20 cm, 20 cm, 45°)

Map quality: [+100, +50, +25] for [Excellent, Good, OK] quality

Timed run:
- 100 points: TBD
- 75 points: TBD
- 50 points: TBD
- 25 points: Finish

The thresholds will be determined by the actual times on the first day of competition.


## Event 2: Maze Explorer [500 points max]

**Level 1**

From the starting position, explore the maze and make a map of the environment. Then return to the start location/pose.

For partial credit:
- You may choose to use botgui for manual exploration of the map via right-clicking (using A* algorithm), but note that this will result in a deduction.
- Alternatively, you can operate the bot manually (tele-op) to construct the map. In this case, points will be awarded based on the map's quality.

**Level 2: Cone Challenge**

You will map the environment as in Level 1, with the addition that we will place 3 randomly colored cones (no duplicates) in the maze. After mapping the environment, you will navigate to the cones in rainbow order (red first) before returning to the start.

**Level 3: Hey, you. You're finally awake**

You will map the environment as in Level 1. **After you have made your map, tell us. You can then stop your code, save your map, recompile, change your configuration, do anything you want in between runs.** After that, an instructor will take the robot out and put it back in at a random location. Your bot will then "wake up" and complete the Cone Challenge as in Level 2. While you can manually start and stop your bot between runs, once your bot is in the maze, everything must be autonomous. Click-to-drive and teleop are not available for this level.

You are allowed and encouraged to save your map and the cone locations, so you can use a cone to initialize your particles then use the walls to refine your particles in localization-only mode.

**Time Limit:** 10 minutes

### Points

- [+200, +150, +100, +50] points for returning to start pose with error limit:
    - 200 points: < (3 cm, 3 cm, 15°)
    - 150 points: < (5 cm, 5 cm, 15°)
    - 100 points: < (10 cm, 10 cm, 30°)
    - 50 points: < (20 cm, 20 cm, 45°)
- [+100, +50, +25] for map quality [Excellent, Good, OK] regardless of finishing
- +100 points for completing the cone challenge (Levels 2 and 3)
- +100 points for completing the task from an unknown initial location (Level 3)
- 25% deduction for using your A* path planner but using click to drive instead of full autonomy (Levels 1 and 2 only)
- 50% deduction for using teleop (Levels 1 and 2 only)

## Event 3: Warehouse [600 points max]

Below is an example of what the warehouse arena will look like.

<a class="image-link" href="/assets/images/botlab/checkpoints/competition.png">
<img src="/assets/images/botlab/checkpoints/competition.png" alt=" " style="max-width:400px;"/>
</a>

The purpose of this event is to locate Apriltag crates and cones and move the crates to designated drop-off zones.

### Setup

**All Levels**

* Apriltag crates with cones on top of them will be placed in random places in the warehouse
  * Cone colors: Red or Blue
  * Red for Apriltag ID 1/2
  * Blue for Apriltag ID 3/4
  * The color and number of crates/cones used will depend on the level chosen
  * The crates will be placed with enough space around them to complete the tasks
* 2 pairs of cones will be placed in fixed places in the warehouse, marking dropoff zones
  * The cones will be placed in pairs denoting "goal lines"
  * Dropoff cone colors: Green and Yellow
* We will provide you with the 3D printed forks that you will attach to your bot and use to move the Apriltag crates

**Level 1**

* 1 red cone and 1 blue cone will be placed on the Apriltag crates

**Level 2**

* 2 red cones will be placed on the Apriltag crates

**Level 3**

* 3 red or blue cones will be placed on the Apriltag crates

### Tasks

You will have to move crates from their initial locations to a point past the goal lines marked by the cones. Below is a diagram of the goal of this event.

<a class="image-link" href="/assets/images/botlab/checkpoints/warehouse_goal.png">
<img src="/assets/images/botlab/checkpoints/warehouse_goal.png" alt="The image depicts a sequence where robots equipped with forklifts move crates topped with cones. The process involves the transfer of these cone-topped crates from one location to another." style="max-width:500px;"/>
</a>

**Level 1**

Drive into an Apriltag crate with a red cone on top of it, then drive into an Apriltag crate with a blue cone on top of it. Running into the crate is allowed.

**Time limit:** 2 minutes

Points:

* [+100, +50] points for the final distance to a crate
  * 100 points: < 5 cm away from the Apriltag
  * 50 points: < 20 cm away
  * No points for > 20 cm away
  * We will judge you based on the closest point your bot was to a crate during the entire run
* 50% deduction for using teleop or click to drive

**Level 2 and 3**

* Move each Apriltag crate to a drop-off zone marked by a pair of cones.
  * Apriltags with **red** cones on top of them go to the **green** drop off zones
  * Apriltags with **blue** cones on top of them go to the **yellow** drop off zones

**Time limit:** 10 minutes

Points:

* +100 points for each crate picked successfully
  * A successful pick is one where the tines of your fork pass through the slots in the bottom of the crate
* +100 points for each crate moved to the correct drop-off zone
  * A successful drop-off is one where the center of the crate ends up past the "goal line" marked by the cones
* 50% deduction for driving around using teleop or click to drive
