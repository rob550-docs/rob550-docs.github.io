<!-- ---
layout: default
title: Systemd Services Guide
parent: How-to Guide
nav_order: 3
last_modified_at: 2023-11-13 13:37:48 -0500
--- -->

> This guide introduces the concepts related to `systemd` and `systemctl`.

### Contents
* TOC
{:toc}

As our professor mentioned in the Discord channel, working with motion_controller in the mbot_autonomy package involves certain steps. When you run the `install.sh` script, it compiles, installs the code, and also manages a service that starts on boot and runs in the background. The `mbot-motion-controller.service` is enabled the first time you run the install script. As a result, the service always automatically start at boot if the system reaches the network-online.target, as defined in the .service file.

To test any changes you make, you have two options:

1. Re-run the `install.sh` script. This will reinstall the program and restart the service.
2. Stop the service to manually run the motion_controller program from the build folder in mbot_autonomy.



You can start the service by running:
```bash
$ sudo systemctl start mbot-motion-controller.service
```

You can stop the service by running:
```bash
$ sudo systemctl stop mbot-motion-controller.service
```

You can disable the service by running: 
```bash
$ sudo systemctl disable mbot-motion-controller.service
```

You can enable the service by running: 
```bash
$ sudo systemctl enable mbot-motion-controller.service
```

This post includes additional information to help you better understand the process.

Note that the content here is specific to Linux systems. systemd is a system and service manager used in many Linux distributions to manage system processes and services. It is not used in non-Linux operating systems like Windows or macOS, which have their own methods of managing services.


### What is a .service File?
A `.service` file is a configuration file used by `systemd` to manage a service (which could be a program or script). This file describes how the service should start, stop, and operate. It includes details like what command to execute to start the service, under which user it should run, what to do if it fails, and more.

```ini
[Unit]
Description=MBot Motion Controller
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
Restart=on-failure
RestartSec=5
ExecStart=mbot_motion_controller
User=root

[Install]
WantedBy=network-online.target
```

Look at the `.service` file for motion controller as an example, this `.service` file defines a systemd service for a "MBot Motion Controller". It ensures that the service starts after the network is online, restarts on failure after a 5-second delay, and runs as the root user. The service is associated with the network-online.target, meaning it's intended to start when the network is fully operational.

### What does it mean by enable/start and disable/stop?