---
layout: default
title: Competition
nav_order: 5
parent: Checkpoints
grand_parent: Armlab
last_modified_at: 2024-10-07 12:00:00 -0500
---

**Updated 1/30/25:** Remove aiming component of basketball launcher.

It's time to showcase your hard work and effort! The competition consists of several challenges that will test every component of your project, from camera calibration to inverse kinematics. Based on your performance on the competition tasks, you are eligible for points of extra credit that are added to your report grade. **The competition is only for extra credit, and your performance cannot negatively affect your grade.**

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
- The first 8 blocks are worth 30 points
- Any blocks after that are worth 50 points
- The following lookup table gives scores for N=1 to N=19 (tying the class record)

| 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 19 |
|:-|:-|:-|:-|:-|:-|:-|:-|:-|:-|:-|:-|:-|:-|:-|:-|:-|:-|:-|
| 30 | 60 | 90 | 120 | 150 | 180 | 210 | 240 | 290 | 340 | 390 | 440 | 490 | 540 | 590 | 640 | 690 | 740 | 790 |

## Event 4: Free throw (400 points)
<a class="image-link" href="/assets/images/armlab/checkpoints/competition1.png">
<img src="/assets/images/armlab/checkpoints/competition1.png" alt="" style="max-width:500px;"/>
</a>

### Task Setup
Design a mechanism that can aim and launch a mini-basketball using the RX200, then score as many points as possible, with a maximum 400 points. 

**Arena**

Multiple basketballs will be in the storage area, RX200 should pick it up autonomously using the camera.

<a class="image-link" href="/assets/images/armlab/checkpoints/competition2.png">
<img src="/assets/images/armlab/checkpoints/competition2.png" alt="" style="max-width:600px;"/>
</a>


**Anchoring**

Red circle below shows the aluminum extrusions that can be used for anchoring. Securely fix your mechanism to the table ensures repeatability. The mechanism should be easily removeable from the arena.

<a class="image-link" href="/assets/images/armlab/checkpoints/competition3.png">
<img src="/assets/images/armlab/checkpoints/competition3.png" alt="" style="max-width:600px;"/>
</a>

**Objective**

Within a limited time, your RX200 robotic arm must retrieve a basketball from storage, load it into your custom-designed shooting mechanism, aim at the hoop, and then launch the basketball.  When the basketball leaves the mechanism, the point of departure should be on the opposite half plane of the hoops. The mechanism can either aim first and then the arm places the ball, or the arm can load the ball before aiming.

### Level Setup
<a class="image-link" href="/assets/images/armlab/checkpoints/competition4.png">
<img src="/assets/images/armlab/checkpoints/competition4.png" alt="" style="max-width:600px;"/>
</a>

The hoop comes in 3 sizes: small, medium, and large.
- Level 1: One basket (any size)
- Level 2: Two baskets (pick 2 from the 3 sizes)
    - [streak rule] you can score in a hoop up to 2 time in a row before switching to a different hoop. For examples:
        - Up-down-up-down is valid: You are alternating between the two hoops with every shot.
        - UU-DD-UU-DD is valid: You score twice in a row in one hoop, then switch and repeat this pattern.
        - UUU-DD is NOT valid: This breaks the rule because you shot three times in a row in one hoop. You’re only allowed two consecutive scored shots before switching hoops.
- Level 3: Three baskets. The streak rule holds.

**Requirements**
1. Choose a level for the task
2. Choose combination of the hoops (if level 2 or 3 selected)
3. The instructor or GSI will set up the hoop location
4. Complete the task within **180 seconds**

### Points
- +20 points for large hoop
- +30 points for medium hoop
- +50 points for small hoop
- +20 bonus points for each two-basket streak
- +40 bonus points for each three-basket streak

Deductions/bonuses for manual/automatic control: The deduction is applied to your score. For bonuses, your score is still capped at 400.
* -20% for manual loading (moving the ball from the holding area to the receptacle)
* -30% for manual arming (storing the potential energy used to fire the ball)
* -20% for manual firing (making the ball shoot)

## Event 5: Bonus Event (200 points)
### Task Setup
<a class="image-link" href="/assets/images/armlab/checkpoints/competition5.png">
<img src="/assets/images/armlab/checkpoints/competition5.png" alt="" style="max-width:300px;"/>
</a>

For this event, the basketball hoops are setup as shown in the figure
- Complete the task in **120 seconds**
- You need to aim and shoot cycle through all hoops

### Points
- Each cycle through all hoops (L-M-S) is worth 100 points. A cycle means you successfully score in the Large, Medium, and Small hoops in sequence.
        - (L-M-S)-(L-M-S) is 100 + 100 = 200 pts
- Incomplete cycle point is calculated based on the points for individual hoops, halved

Deductions/bonuses for manual/automatic control: The deduction is applied to your score. For bonuses, your score is still capped at 400.
* -20% for manual loading (moving the ball from the holding area to the receptacle)
* -30% for manual arming (storing the potential energy used to fire the ball)
* -20% for manual firing (making the ball shoot)

## Score Calculation
Your overall score will be the sum of your best run on each event. Each event can be completed multiple times at different levels.
- The 3 top scoring teams **in each section** will receive bonus points on the report: [+3, +2, +1] points. 
- The top score **in each event** will receive +1 bonus point on the report for the team.

Your score for each event is based on the final layout of the playing field. You have the full time-limit for each event, but do not have to use the full time. If you complete the task before the time-limit, or wish to “break” your code to stop the robot (e.g., to keep the arm safe, or to prevent it from knocking over blocks), you may do so, but will be judged at that final configuration.
