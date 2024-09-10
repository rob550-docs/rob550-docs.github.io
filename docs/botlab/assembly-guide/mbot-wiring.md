---
layout: default
title: Mbot Wiring
parent: Assembly Guide
grand_parent: Botlab
nav_order: 4
last_modified_at: 2024-09-09 14:37:48 -0500
---

{: .important}
This guide is for Differential Drive MBot Classic.

### Contents
* TOC
{:toc}


## Assemble the jumper wires

| Components     | #         | 
|:-------------|:-----------|
| jumper wires (Black/Red/Yellow/Blue/Green/White)   |6| 
| Crimp connector housings (3/4/6 pins)   |3| 

You need to build the OLED/Bootload cable assembly which is shown. This cable plugs into the Jetson or RasPi 40pin header, the OLED module, and the Control board and allows both communicating with the OLED and controlling the run and bootload modes on the control board.

Gather the components listed in the table and ensure you have all the items shown in the image below. Assemble a 3-heads wire as depicted in the second image. **Color and order matters!** Your assembled wire should look exactly like the image indicated.

<div class="popup-gallery">
<a href="/assets/images/botlab/assembly/middlePlate/mbot_jumper_wires.jpg" title="Assemble the jumper wires 1"><img src="/assets/images/botlab/assembly/middlePlate/mbot_jumper_wires.jpg" width="200" height="200"></a>
<a href="/assets/images/botlab/assembly/middlePlate/pi5_jumper_wires_assemble.jpg" title="Assemble the jumper wires 2"><img src="/assets/images/botlab/assembly/middlePlate/pi5_jumper_wires_assemble.jpg" width="400" height="200"></a>
</div>

## Final Wiring
1. Connect all the cables and power

    | Components     | #         | 
    |:-------------|:---- -------|
    |  12V Power Bank |1| 
    |  DC power “Y” cable |1| 
    | USB-C Cable |2| 

    1. Plug in WiFi dongle to Jetson
    2. Use the "Y" Cable to connect the power bank and the Robotics Control Board
    3. Use the USB Micro Cable to connect the power bank and the Jetson
    4. Use the USB-C Cable to connect the Robotics Control Board and Jetson
    5. Plug in the Lidar's USB to the Jetson

    <a class="image-link" href="/assets/images/botlab/assembly/final/wiring.png">
        <img src="/assets/images/botlab/assembly/final/wiring.png" alt="" style="max-width:600px;"/>
    </a> 

Yay! Now you have a complete version of MBot!
