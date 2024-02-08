---
layout: default
title: Competition
nav_order: 5
parent: Checkpoints
grand_parent: Armlab
last_modified_at: 2024-01-18 12:00:00 -0500
---

It's time to showcase your hard work and effort!

Update 2/7: Edits for clarity. Also change some point values.

<a class="image-link" href="https://i.imgflip.com/3t9jkg.jpg"><img src="https://i.imgflip.com/3t9jkg.jpg" alt="" style="max-width:400px;"/>
</a>

### Contents
* TOC
{:toc}


## Event 1: Pick ‘n sort! (300 points)
### Task Setup
Based on your selected level, specific blocks (cubes) will be placed on the positive half-plane (in front of the robotic arm). Your team need to sort small blocks to the left and large ones to the right of the arm in the negative half plane.

Requirements:
- Choose a level for the task.
- Complete the task within 180 seconds.

### Level Setup
- Level 1 - 3 large blocks (R G and B), not stacked
- Level 2 - 6 blocks, 3 of each size, random colors (ROYGBV), not stacked
- Level 3 - 9 blocks, random sizes, random colors (ROYGBV), possibly stacked two high

### Points
- +30 points for each block picked and dropped correctly
- +(10 * level) points for completing the task in the time allotted
- 50% deduction for using “click” for each pick-and-place instead of an autonomous approach


## Event 2: Pick n’ stack! (300 points)
### Task Setup
Blocks (cubes) will be placed within the region defined by AprilTags 1-4 on the board. The objective is to stack the blocks on one or more of the AprilTags.

Requirements:
- Choose a level for the task.
- Complete the task within 120 seconds.
- Stack all blocks to a height of three, on top of one or more AprilTags.
- The robot's end-effector must remain within the cubic rectangular space defined by the AprilTags and a height limit of 30cm.

### Level Setup
- Level 1 - 3 large blocks (R G and B), not stacked.
- Level 2 - 6 blocks, 3 of each size, random colors (ROYGBV), not stacked.
- Level 3 - 6 blocks, 3 of each size, random colors (ROYGBV), possibly stacked two high.

### Points
- +40 points for each block successfully picked and stacked.
- +(20 * level) points for completing the task in the time allotted.
- 50% deduction for using “click” for each pick-and-place instead of an autonomous approach.


## Event 3: Line ‘em up! (500 points)
### Task Setup
Blocks will be placed in a specific configuration based on your selected level, and your team must arrange them in rainbow color order horizontally. 

Requirements:
- Choose a level for the task.
- Complete the task within 600 seconds.
- For Level 1, align only the large blocks in this sequence. 
- For Levels 2 and 3, arrange both large and small blocks in separate lines, each following the rainbow color order.
- Block centers must be within 3 cm of a straight line.
- Each line's length must be < 30 cm.

### Level Setup
- Level 1 - only big blocks (6, ROYGBV)
- Level 2 - big (6, ROYGBV) & small blocks (6, ROYGBV). Possibly stacked but no more than four blocks high.
- Level 3 - 12 blocks (6 small, 6 large, ROYGBV), with additional "distractor" objects. Possibly stacked but no more than four blocks high. Your objective for level 3 is to line up the small and large blocks (cubes) as you did in level 2, but avoiding any other shaped block on the table.

### Points
- +30 points for each block in the correct place & order 
- +10 points if the block is out of order or the center is >3cm from the line but still clearly “lined up”
- +20 bonus points for the neatness of each line (centers are perfectly aligned and/or spacing is minimal)
- +100 points for performance with distractors (level 3)
- 50% deduction for using "click" for each pick-and-place instead of an autonomous approach.

## Event 4: Stack ‘em high! (600 points)
### Task Setup
Blocks will be placed in a specific configuration based on your selected level, and your team needs to stack up the blocks in rainbow color order vertically. 

Requirements:
- Choose a level for the task.
- Complete the task within 600 seconds.
- For Level 1, align only the large blocks in rainbow sequence (red on bottom).
- For Level 2, Stack up large and small blocks in separate stacks, each following the rainbow color order.
- For Level 3, Stack up small blocks in rainbow color order in one stack. Separately, stack the semi-circle and arch blocks so that they form a rectangle when put together (order = semi-circle, arch, semi-circle, arch, and so on). Stack in rainbow color order.

### Level Setup
- Level 1 - only large blocks (6, ROYGBV)
- Level 2 - big (6, ROYGBV) & small blocks (6, ROYGBV). Possibly stacked but no more than four blocks high.
- Level 3 - small blocks (6, ROYGBV), and semi-circle (3) & arch blocks (3). Small blocks are possibly stacked but no more than four blocks high. Semi-circles and arches will not be stacked.

### Points
- For each stack, the first block is worth 10 points, and each additional block is worth (n*10) points (210 points per stack)
- +40 points for each completed stack in the correct order
- +100 points for performance with arches/semicircles
- 50% deduction for using “click” for each pick-and-place instead of an autonomous approach.

## Bonus Event: To the sky!
### Task Setup
For this task, use only large blocks and stack them in any color order you choose. You have the freedom to add any number of blocks to the board, but they must not be pre-stacked. You can add blocks to the board as the arm is moving. Your goal is to stack them as high as possible, pushing the limit of how many blocks you can stack. The challenge is to see just how high your stack can reach!

**You cannot use “click” to pick and place.**

### Points
- The first block is worth 10 points, and each additional block is worth (n*10) points

## Score Calculation
Your overall score will be the sum of your best run on each event. Each event can be completed multiple times at different levels.
- The 3 top scoring teams **in each section** will receive bonus points on the report: [+3, +2, +1] points. 
- The top score **in each event** will receive +1 bonus point on the report for the team.

Your score for each event is based on the final layout of the playing field. You have the full time-limit for each event, but do not have to use the full time. If you complete the task before the time-limit, or wish to “break” your code to stop the robot (e.g., to keep the arm safe, or to prevent it from knocking over blocks), you may do so, but will be judged at that final configuration.
