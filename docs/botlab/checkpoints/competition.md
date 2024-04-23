---
layout: default
title: Competition
nav_order: 7
parent: Checkpoints
grand_parent: Botlab
last_modified_at: 2024-04-23 16:37:48 -0500
---

UPDATE 4/23/24: Clarify exactly what is needed for Tasks 1 and 4.

<a class="image-link" href="/assets/images/botlab/checkpoints/doge-meme.png">
<img src="/assets/images/botlab/checkpoints/doge-meme.png" alt=" " style="max-width:250px;"/>
</a>

### Contents
* TOC
{:toc}

### Event 1: Speed Run [200 points max]

You will complete two runs in total, one of which is judged for accuracy and the second which is judged for speed.

For the first run, make a circuit around a convex arena by driving the path given to you. Do this while running SLAM to make a map of the arena. Return to the starting pose. You are judged based on the final error in pose and your map quality. Save an image of your map and upload the image to Google Drive.

For the second run, drive quickly around the path. You are judged on the speed to complete the run. For this run, you will not be judged based on your final error, but your ending pose must be close enough to the start that you are still partially on the starting piece of paper.

Error in pose after mapping:
- 100 points: < (3 cm, 3 cm, 15°)
- 75 points: < (5 cm, 5 cm, 15°)
- 50 points: < (10 cm, 10 cm, 30°)
- 25 points: < (20 cm, 20 cm, 45°)

Timed run:
- 100 points: < time TBD
- 75 points: < time TBD
- 50 points: < time TBD
- 25 points: Finish


### Event 2: Heavy Lift [300 points max]

Begin from your initial position and use the camera along with AprilTags to identify a target (any order). Once located, drive towards the target, collect it, and then proceed to a designated drop-off location, i.e., a target to stack it on. Designated drop-off targets will be given on the day of the competition.

Continue this process for any additional blocks, and after completing the task with all targets, return to your starting position. This entire activity can be efficiently executed using only the camera and visual-servoing techniques. We will position 1 to 3 targets within a defined area of 1 meter.


Points:
- +50 points - each per target lifted
- +50 points - each per target successfully stacked

### Event 3: Maze Explorer [400 points max]

From the starting position, explore the maze and make a map of the environment. Then return to the start location/pose. Save the map to an image file and upload your map to Google Drive. Once all groups finish the task, the maps will be ranked according to accuracy. 

For partial credit:
- You may choose to use botgui for manual exploration of the map via right-clicking (using A* algorithm), but note that this will not earn you points for autonomous navigation.
- Alternatively, you can operate the bot manually (tele-op) to construct the map. In this case, points will be awarded based on the map's quality. 

Points:
- [+100, +75, +50, +25] points for returning to start pose with error limit:
    - 100 points: < (3 cm, 3 cm, 15°)
    - 75 points: < (5 cm, 5 cm, 15°)
    - 50 points: < (10 cm, 10 cm, 30°)
    - 25 points: < (20 cm, 20 cm, 45°)
- [+100, +50, +25] for map quality [Excellent, Good, OK] regardless of finishing
- +100 points for using your path planner (click to drive)
- +100 points for completing a full map fully autonomously

### Event 4: Warehouse [500 points max]

Below is an example of what the warehouse arena will look like. Your MBot will move the pick-up targets (green boxes, arrow indicating pick up direction) to the drop-off targets (red boxes, to be stacked on top of). Your bot will start at a random initial location and must localize itself within the map at the start.

<a class="image-link" href="/assets/images/botlab/checkpoints/competition.png">
<img src="/assets/images/botlab/checkpoints/competition.png" alt=" " style="max-width:400px;"/>
</a>

Points:
- [+100 points] Level 1: from the starting location, retrieve a single green target and stack it on a red drop-off target within 3 minutes.
- [+150 points] Level 2: retrieve 2 green targets and stack each on a drop-off location within an additional 3 minutes (6 minutes total).
- [+250 points]  Level 3: retrieve 4 green targets and stack each on a drop-off location within an additional 6 minutes (12 minutes total).
