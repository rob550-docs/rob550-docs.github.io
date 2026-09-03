---
layout: default
title: Troubleshooting
nav_order: 6
grand_parent: Armlab
parent: How-to Guide
last_modified_at: 2026-09-02 12:00:00 -0400
---

> Symptoms students hit most often, and how to work through them. This page will grow as the term goes on.

### Contents
* TOC
{:toc}

## The control station says "Arm offline"

The GUI opens, but the status bar reports the arm as offline and the gripper buttons stay greyed out. The arm talks to the laptop over **Ethernet**, so this is almost always a network problem rather than a code problem.

Work through these in order:

**1. Check which port the cable is in.**

{: .warning}
The arm must be plugged into the **USB-to-Ethernet adapter**, not the laptop's built-in Ethernet port. The built-in port is reserved for internet access and is on a different IP address and subnet, so an arm plugged in there is unreachable. See [Hardware](/docs/armlab/hardware#power-and-network).

**2. Check the arm has power.** The 24 V supply is mounted under the board. The arm should be powered up and finished booting before you start the control station.

**3. Ping the arm.** Use the IP printed on the label on the back of the arm:

```bash
ping 192.168.1.xxx
```

- **Replies:** the network is fine — go to step 4.
- **No replies:** cable, adapter, port, or power. Nothing in your Python will fix it.

**4. Check the IP in the code matches the arm.** `XARM_IP` is set near the top of `src/lite6arm.py`, and each station's arm is different:

```bash
grep XARM_IP src/lite6arm.py
```

**5. Check your network interfaces.** `ip addr` should show the adapter with an address on the same subnet as the arm.

## The control station says "Camera offline"

The video panel shows *No Video Input* instead of a live image.

**1. Check the USB cable**, at both ends. Then confirm the laptop sees the device at all:

```bash
lsusb
```

Look for an Intel RealSense entry. If nothing appears, it is the cable or the port — try a different one.

**2. Check the connection type.** The camera needs USB 3.x:

```bash
rs-enumerate-devices -s
```

{: .warning}
If this reports **USB 2.1**, the high-resolution profiles the control station asks for are unavailable and the stream will fail to start. Try a different port and a different cable before asking an instructor for help.

**3. Check nothing else is holding the camera.** Only one process can open it at a time. `realsense-viewer` is the usual culprit, followed by a control station you thought you closed:

```bash
ps aux | grep -e realsense -e python
```

Kill anything left over, then try again. See [Linux Command Line Tools](/docs/armlab/how-to-guide/linux-clt#managing-running-programs).

**4. Confirm the camera itself works.** Close the control station and open the viewer:

```bash
realsense-viewer
```

If the stream looks correct there, the camera is fine and the problem is in your setup or your code. See the [Camera Guide](/docs/armlab/how-to-guide/camera-guide).

## `ImportError` on `pyrealsense2`, `cv2` or `xarm`

Check the prompt for `(env550lab)` before anything else — running with the system Python instead of the lab environment produces exactly this error.

```bash
source ~/.bashrc          # the RealSense binding is found through PYTHONPATH
conda activate env550lab  # if the prompt still does not show it
```

See [Software](/docs/armlab/software#environment-notes) for why `pyrealsense2` lives outside the environment.
