---
layout: default
title: Competition
nav_order: 5
parent: Checkpoints
grand_parent: Armlab
last_modified_at: 2024-08-26 12:00:00 -0500
---

It's time to showcase your hard work and effort!

<a class="image-link" href="https://i.imgflip.com/3t9jkg.jpg"><img src="https://i.imgflip.com/3t9jkg.jpg" alt="" style="max-width:400px;"/>
</a>

### Contents
* TOC
{:toc}


## Event 1: Sort 'n stack! (300 points)
### Task Setup
Based on your selected level, specific blocks (cubes) will be placed on the positive half-plane (in front of the robotic arm). Your team needs to sort small blocks to the left and large ones to the right of the arm in the negative half plane. Depending on the level, you may need to stack them as well.

Requirements:
- Choose a level for the task.
- For levels 2 and 3, the blocks must be stacked in rainbow order (red on bottom).
- Complete the task within 180 seconds.

### Level Setup
- Level 1 - 3 large blocks (R G and B), not stacked
- Level 2 - 6 blocks, 3 of each size, random colors (ROYGBV), not stacked
- Level 3 - 6 blocks, 3 of each size, random colors (ROYGBV), possibly stacked two high, with additional "distractor" objects. Your objective for level 3 is to sort and stack the small and large blocks (cubes) as you did in level 2, but avoiding any other shaped block on the table.

### Points
- +30 points for each block picked and dropped correctly
- +(10 * level) points for completing the task in the time allotted
- +90 points for performance with distractors (level 3)
- 50% deduction for using “click” for each pick-and-place instead of an autonomous approach


## Event 2: Line ‘em up! (500 points)
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
- Level 2 - big (6, ROYGBV) & small blocks (6, ROYGBV). Possibly stacked but no more than two blocks high.
- Level 3 - 12 blocks (6 small, 6 large, ROYGBV), with additional "distractor" objects. Possibly stacked but no more than three blocks high. Your objective for level 3 is to line up the small and large blocks (cubes) as you did in level 2, but avoiding any other shaped block on the table.

### Points
- +30 points for each block in the correct place & order 
- +10 points if the block is out of order or the center is >3cm from the line but still clearly “lined up”
- +20 bonus points for the neatness of each line (centers are perfectly aligned and/or spacing is minimal)
- +100 points for performance with distractors (level 3)
- 50% deduction for using "click" for each pick-and-place instead of an autonomous approach.


## Event 3: To the sky!
### Task Setup
For this task, use only large blocks and stack them in any color order you choose. You have the freedom to add any number of blocks to the board, but they must not be pre-stacked. You can add blocks to the board as the arm is moving. Your goal is to stack them as high as possible, pushing the limit of how many blocks you can stack. The challenge is to see just how high your stack can reach!

**You cannot use "click" to pick and place.**

Requirements:
- Complete the task within 600 seconds.

### Level Setup
- You place the blocks on the board.
- The blocks you place must not be stacked.
- Blocks can be placed while the arm is moving.

### Points
- Given N blocks stacked, the score for a stack is computed according to the following formula, then rounded to the nearest whole multiple of 10:
- SCORE = 50 * EXP(0.15*N)
- The following lookup table gives scores for N=1 to N=19 (tying the class record)

| 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 19 |
|:-|:-|:-|:-|:-|:-|:-|:-|:-|:-|:-|:-|:-|:-|:-|:-|:-|:-|:-|
| 60 | 70 | 80 | 90 | 110 | 120 | 140 | 170 | 190 | 220 | 260 | 300 | 350 | 410 | 470 | 550 | 640 | 740 | 860 |


## Score Calculation
Your overall score will be the sum of your best run on each event. Each event can be completed multiple times at different levels.
- The 3 top scoring teams **in each section** will receive bonus points on the report: [+3, +2, +1] points. 
- The top score **in each event** will receive +1 bonus point on the report for the team.

Your score for each event is based on the final layout of the playing field. You have the full time-limit for each event, but do not have to use the full time. If you complete the task before the time-limit, or wish to “break” your code to stop the robot (e.g., to keep the arm safe, or to prevent it from knocking over blocks), you may do so, but will be judged at that final configuration.
