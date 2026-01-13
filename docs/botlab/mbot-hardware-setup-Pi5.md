---
layout: default
title: MBot Hardware Setup
parent: Botlab
nav_order: 1
last_modified_at: 2026-01-13 12:20:00 -0500
---

{: .important}
This guide is for **ROS2** MBot Classic running **firmware v1.1.4 and above**.

Your MBot may arrive fully assembled or in three parts.
- If it is not fully assembled, follow the [MBot Assembly Guide](https://mbot.robotics.umich.edu/docs/hardware/classic/assembly/) and complete Steps 1–3 only. Stop before the final wiring.

At this point, your MBot is mechanically assembled but not wired. Follow this page to complete the remaining wiring.

### Extra Wiring Step: LiDAR Power Update

1. Find the LiDAR power cable in the lab.

    <a class="image-link" href="/assets/images/botlab/hardware-setup/lidar0.png">
    <img src="/assets/images/botlab/hardware-setup/lidar0.png" alt="LiDAR cable" style="max-width:300px;"/>
    </a>
2. Remove the top plate that holds the LiDAR.
3. On the LiDAR board, find the 3-pin power connector. See images below.

    <div class="popup-gallery">
        <a href="/assets/images/botlab/hardware-setup/lidar2.png" title=""><img src="/assets/images/botlab/hardware-setup/lidar2.png" width="300" height="200"></a>
        <a href="/assets/images/botlab/hardware-setup/lidar3.jpg" title=""><img src="/assets/images/botlab/hardware-setup/lidar3.jpg" width="300" height="200"></a>
    </div>
4. Unplug the original 3-pin cable (the one marked by the green arrow in the image).
5. Plug in the new cable from Step 1. The connector only fits one way. The colors shown in the image correspond to these pins, but your cable colors may be different:
    - Yellow - GND
    - Blue - CTRL
    - Red - 5V

    <a class="image-link" href="/assets/images/botlab/hardware-setup/lidar4.jpg">
    <img src="/assets/images/botlab/hardware-setup/lidar4.jpg" alt="" style="max-width:300px;"/>
    </a>

That’s it for the top plate.


### Final Wiring
Now go to the official [MBot Wiring Guide](https://mbot.robotics.umich.edu/docs/hardware/classic/assembly/mbot-wiring/) and finish all the wiring.

After that, also connect the LiDAR to the bottom board:
1. Find the `SV3` slot on the bottom board (see image).
2. Connect each wire to the same signal used on the LiDAR board. In the example shown:
    - Blue - CTRL (marked as `_|‾|_`.)
    - Red - 6V
    - Yellow - GND

    <a class="image-link" href="/assets/images/botlab/hardware-setup/lidar5.jpg">
    <img src="/assets/images/botlab/hardware-setup/lidar5.jpg" alt="" style="max-width:300px;"/>
    </a>

Once assembled, complete the system setup by following the steps in the [MBot System Setup](mbot-system-setup-Pi5).

---

**Important:** Do not power on the MBot before completing the system setup. The SD card you received might not be empty or could have the wrong operating system installed. Powering on the device before checking could damage the Raspberry Pi.
{: .text-red-200}

If you've already powered on the Raspberry Pi and notice the LED light has "4 long flashes followed by 5 short flashes", it indicates a [Fatal Firmware Error](https://www.raspberrypi.com/documentation/computers/configuration.html#led-warning-flash-codes). You can find the solution in the troubleshooting guide [here](/docs/botlab/how-to-guide/pi-troubleshooting).

