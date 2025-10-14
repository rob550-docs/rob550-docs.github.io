---
layout: default
title: MBot Hardware Setup
parent: Botlab
nav_order: 1
last_modified_at: 2025-10-14 11:20:00 -0500
---

{: .important}
This guide has been updated for **ROS2** MBot Classic!

Your MBot may come fully assembled or in three main parts. Use the official MBot [Assembly Guide](https://mbot.robotics.umich.edu/docs/hardware/classic/assembly/) as a reference to put yours together. Just follow steps 1, 2, and 3, but stop before the final wiring. There are extra steps you need to do first.

### Extra Wiring Step: LiDAR Power Update

1. Find the following package of wires in the lab.

    <a class="image-link" href="/assets/images/botlab/hardware-setup/lidar0.png">
    <img src="/assets/images/botlab/hardware-setup/lidar0.png" alt="LiDAR cable" style="max-width:300px;"/>
    </a>
2. Remove the top plate where the LiDAR is mounted.
3. Look at the board on LiDAR, there are 2 slots directly connected to the USB interface thru the wires. The detail pin definition is shown in the image below.

    <div class="popup-gallery">
        <a href="/assets/images/botlab/hardware-setup/lidar2.png" title=""><img src="/assets/images/botlab/hardware-setup/lidar2.png" width="300" height="200"></a>
        <a href="/assets/images/botlab/hardware-setup/lidar3.jpg" title=""><img src="/assets/images/botlab/hardware-setup/lidar3.jpg" width="300" height="200"></a>
    </div>
4. Unplug the original black/blue/red cable.
5. Plug in the yellow/blue/red cable from Step 1:
    - Yellow - GND
    - Blue - CTRL
    - Red - 5V

    <a class="image-link" href="/assets/images/botlab/hardware-setup/lidar4.jpg">
    <img src="/assets/images/botlab/hardware-setup/lidar4.jpg" alt="" style="max-width:300px;"/>
    </a>

That’s it for the top plate.

---

### Final Wiring
Now go back to the official MBot Wiring Guide and finish the wiring.
In addition to everything in the guide, you also need to connect the LiDAR to the bottom board:
1. Find the `SV3` slot on the bottom board (see image).
2. Connect the yellow wire to GND, red to 6V, blue to CTRL which marked as `_|‾|_`.

    <a class="image-link" href="/assets/images/botlab/hardware-setup/lidar5.jpg">
    <img src="/assets/images/botlab/hardware-setup/lidar5.jpg" alt="" style="max-width:300px;"/>
    </a>


Once assembled, complete the system setup by following the steps in the [MBot System Setup](mbot-system-setup-Pi5).

---

**Important:** Do not power on the MBot before completing the system setup. The SD card you received might not be empty or could have the wrong operating system installed. Powering on the device before checking could damage the Raspberry Pi.
{: .text-red-200}

If you've already powered on the Raspberry Pi and notice the LED light has "4 long flashes followed by 5 short flashes", it indicates a [Fatal Firmware Error](https://www.raspberrypi.com/documentation/computers/configuration.html#led-warning-flash-codes). You can find the solution in the troubleshooting guide [here](/docs/botlab/how-to-guide/pi-troubleshooting).

