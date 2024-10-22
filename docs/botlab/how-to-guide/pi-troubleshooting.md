---
layout: default
title: Pi Troubleshooting
parent: How-to Guide
grand_parent: Botlab
nav_order: 4
last_modified_at: 2024-10-22 17:37:48 -0500
---

LED warning flash codes
### Contents
- TOC
{:toc}

### Fatal firmware error
> The LED light, labeled "STAT," is located next to the SD slot. "4 long flashes followed by 5 short flashes" indicates [Fatal Firmware Error](https://www.raspberrypi.com/documentation/computers/configuration.html#led-warning-flash-codes).

What You'll Need:
- SD card reader
- Laptop

1. Download Raspberry Pi Imager from [Raspberry Pi Official Website](https://www.raspberrypi.com/software/).
2. Open the Imager and select the following options:
- Raspberry Pi Device: "Raspberry Pi 5"
- Operating System: "Misc utility images" > "Bootloader (Pi 5 family)" > "SD Card Boot"
- Choose Storage: Select your SD card
3. Once the SD card is flashed, insert it into your Raspberry Pi.
4. Turn the power on and check if the Raspberry Pi is working:
    1. External Monitor Method: Connect your Pi to a monitor and power it on. If the screen turns completely green, your Pi is back to life.
    2. LED Light Method: Power on the Pi and watch the LED. If the green light blinks continuously, your Pi is functioning properly.

Now, you can turn off the battery pack and restart the system setup guide from the beginning.

