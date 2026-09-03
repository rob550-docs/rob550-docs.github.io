---
layout: default
title: How to Run the Control Station
nav_order: 2
grand_parent: Armlab
parent: How-to Guide
last_modified_at: 2026-09-02 12:00:00 -0400
---

> Quick reference for launching the control station. The [Setup Guide](/docs/armlab/setup-guide#testing) walks through your first run with screenshots; this page is what to come back to afterwards. For what the code does once it starts, see [Software](/docs/armlab/software).

### Contents
* TOC
{:toc}

## Launching

```bash
cd src
python control_station.py          # real arm (default)
python control_station.py --sim    # MuJoCo simulation
```

`--sim` needs the simulator already running in its own terminal:

```bash
mujoco-sim
```

| Flag | Effect |
| ---- | ------ |
| `--real` | Connect to the arm hardware. This is the default |
| `--sim` | Drive the MuJoCo simulation over LCM instead |

The two are mutually exclusive. Both need the full stack installed, so simulation is not a way around a broken camera install.

## Before you energize the arm

1. The workspace is clear and nobody is reaching into it.
2. You know where the E-stop is.
3. The arm and camera cables are connected, and `realsense-viewer` is **closed** — it holds the camera and the control station will not get it.
4. Speed slider low if you are about to run new code.

## Shutting down

{: .warning}
Click **Sleep Arm** before closing the GUI. It returns the arm home and then cuts motor power. Closing the window on its own leaves the arm energized where it stands.

Then, in order: close the GUI window, and `ctrl+c` the simulator in its terminal if you were running one.

## When it will not start

| Symptom | Cause |
| ------- | ----- |
| `ImportError` on `pyrealsense2`, `cv2` or `xarm` | Terminal is not in `env550lab`, or `~/.bashrc` has not been sourced |
| Camera offline in the status bar | `realsense-viewer` or a stale Python process holds the device |
| Arm offline in the status bar | Wrong `XARM_IP`, or the arm is unreachable — `ping` it first |
| Qt `xcb` plugin error | Broken environment; see the repo's `install_scripts/README.md` |

More detail in [Troubleshooting](/docs/armlab/how-to-guide/troubleshooting) and [Linux Command Line Tools](/docs/armlab/how-to-guide/linux-clt#managing-running-programs).
