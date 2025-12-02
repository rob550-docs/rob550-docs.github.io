---
layout: default
title: Design Lab
nav_order: 6
parent: Checkpoints
grand_parent: Botlab
last_modified_at: 2025-10-30 12:37:48 -0500
---

### Contents
* TOC
{:toc}

## Objective: Autonomous warehouse robot

Design and build a forklift mechanism for the MBot to autonomously locate, pick up, and deliver pallets in a warehouse setup.

The forklift should be 3D printed, servo-actuated, and mountable to the MBot chassis. Your robot will use its camera and LiDAR to navigate around traffic cones, and deliver cargo to designated storage areas.

The final system will integrate mechanical design, servo control, and vision-based perception for autonomous operation.

### System overview
<a class="image-link" href="/assets/images/botlab/checkpoints/design-lab01.png">
<img src="/assets/images/botlab/checkpoints/design-lab01.png" alt=" " style="max-width:400px;"/>
</a>

### Example photos from past semester projects
<a class="image-link" href="/assets/images/botlab/checkpoints/design-lab02.png">
<img src="/assets/images/botlab/checkpoints/design-lab02.png" alt=" " style="max-width:600px;"/>
</a>


## Provided Materials

### MBot CAD (STEP) file
{: .py-2 }
[Open Model in new tab](https://umich2673.autodesk360.com/g/shares/SH30dd5QT870c25f12fcdf2d18fed05cc396){: .btn target="_blank" } <- Use this link to download any file format you prefer.
{: .py-1 }

<iframe src="https://umich2673.autodesk360.com/shares/public/SH30dd5QT870c25f12fcdf2d18fed05cc396?mode=embed" width="600" height="400" allowfullscreen="true" webkitallowfullscreen="true" mozallowfullscreen="true" frameborder="0"></iframe>


### Servo control library

We provide XL320 servos.
The assembly guide, library, and example code are available in: [mbot_xl320_lib Guide](/docs/botlab/how-to-guide/mbot-xl320-guide).


## Deliverables
1. Phase 1: 3D CAD model of your forklift assembly (due with checkpoint 2)
2. Phase 2: Working prototype mounted on MBot (testing and improving)
3. Phase 3: Autonomous pick-up and delivery (competition)

{: .required_for_report } 
Document your forklift design, including:
<br> 1. Detailed drawings
<br> 2. Bill of Materials (BOM)
<br> 3. Step-by-step instructions:
<br>&nbsp;&nbsp;&nbsp;&nbsp; - How is it assembled?
<br>&nbsp;&nbsp;&nbsp;&nbsp; - How does the mechanism work?
<br>&nbsp;&nbsp;&nbsp;&nbsp; - How do the servos coordinate and operate together?