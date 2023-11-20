---
layout: default
title: System Service Guide
parent: How-to Guide
nav_order: 3
last_modified_at: 2023-11-20 16:45:48 -0500
---

> This guide introduces the system services we used in the mbot repositories.

### Contents
* TOC
{:toc}

Working with motion_controller in the mbot_autonomy package, when you run the `install.sh` script, it compiles, installs the code, and also manages a service that starts on boot and runs in the background. The `mbot-motion-controller.service` is enabled the first time you run the install script, and as a result, the service always automatically start at boot if the system reaches the network-online.target, as defined in the .service file. This applies to other programs that are running as services.

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
### What is systemctl?
- **Services** are background processes that managed by systemd
- **systemd** is a system and service manager for Unix-like operating systems. It is not used in non-Linux operating systems like Windows.
- **systemctl** is the command-line interface tool that you use to send instructions to systemd to manage specific units, like starting or stopping services, enabling them to start on boot, or disabling them. So, when you're using systemctl, you're instructing systemd on how to manage these individual units.

### What is the difference between enable/start and disable/stop?
- **start**: Start a service immediately.
- **enable**: Turn on the service so that it starts automatically at boot time. Note that `enable` does not start the service in the current session, it only configures it to start on subsequent boots.
- **stop**: Stops a running service immediately.
- **disable**: Turn off the automatic starting of the service at boot time. Note that `disable` does not stop the service if it is currently running, it only prevents it from starting during the boot process in the future.

### What is a .service File?
A `.service` file is a configuration file used by `systemd` to manage a service (which could be a program or script). This file describes how the service should start, stop, and operate. It includes details like what command to execute to start the service, under which user it should run, what to do if it fails, and more.
