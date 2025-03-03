---
layout: default
title: Getting Started
parent: Botlab
nav_order: 1
last_modified_at: 2025-03-03 11:20:00 -0500
---

To get started with MBot, both hardware and software set-ups are required. Please follow the given instructions for detailed guidance.

### Contents
- TOC
{:toc}

### Hardware Assembly
If you are given an unassembled MBot, follow the assembly instructions provided on the official MBot website [Assembly Guide](https://mbot.robotics.umich.edu/docs/hardware/classic/assembly/).

If you are given an assembled MBot, you can move to the next step.

**Important:** Do not power on the MBot before following the system setup guide. The SD card you receive might not be empty or could have the wrong operating system installed. Powering on the device before checking could damage the Raspberry Pi.
{: .text-red-200}

If you've already powered on the Raspberry Pi and notice the LED light has "4 long flashes followed by 5 short flashes", it indicates a [Fatal Firmware Error](https://www.raspberrypi.com/documentation/computers/configuration.html#led-warning-flash-codes). You can find the solution in the troubleshooting guide [here](/docs/botlab/how-to-guide/pi-troubleshooting).

### System Setup
After assembling your MBot, complete the system setup by following the steps in the [System Setup Guide](mbot-system-setup-Pi5).