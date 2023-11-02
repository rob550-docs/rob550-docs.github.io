---
layout: default
title: What's new - 11/02
parent: Student Guide
nav_order: 1
last_modified_at: 2023-11-02 10:03:48 -0500
---

- TOC
{:toc}

## What's new - 11/02
New LCM types have been added to [mbot_lcm_base](https://gitlab.eecs.umich.edu/rob550-f23/mbot_lcm_base) for [mbot_autonomy](https://gitlab.eecs.umich.edu/rob550-f23/mbot_autonomy) use. Students need to pull the updates to their forked lcm repo, and then run the following commands to re-compile and install the new lcm types.
```bash
$ cd ~/mbot_ws/mbot_lcm_base
$ ./scripts/install.sh
```

After this update, please proceed to the "Install the Rest of the MBot Code" section and follow the steps provided.

---

## What's new - 10/25
**Update 1: system utility**

Some system utility files have been updated. To retrieve these updates from upstream, please execute the following commands.

```bash
$ cd ~/mbot_sys_utils
$ git pull
$ tail -n 1 mbot_config.txt | sudo tee -a /boot/firmware/mbot_config.txt > /dev/null 
```
To test, run the following and you should see the output "autostart=run".
```bash
$ tail -n 1 /boot/firmware/mbot_config.txt && echo ""
```
Next, Install the services needed to start the networking and report the robot’s IP
```bash
$ cd ~/mbot_sys_utils/services
$ ./install_mbot_services.sh
```

{: .important }
To students who have tried to flash .uf2 file: <br> When you start the robot, it runs the latest flashed UR2 file. If you have only flashed `mbot_calibrate_classic.uf2` and not `mbot.uf2` yet, the robot will perform the calibration routine each time it restarts, be careful that the robot might start moving in such case.

Next turn off the power and turn it back on.
To check if the updates are loaded successfully, run
```bash
$ code /var/log/mbot/mbot_start_networking.log
```
At the bottom of this log file, if you see the message 'Autostart is set to run,' this means your update was successful.
```
===== 2023-10-24 22:09:19 =====
hostname set to 'mbot-0000-example'
Connected to active WiFi network 'exmaple'. Done.
Autostart is set to run 
```

**Update 2: firmware**

- If you just fisnihed **Set up Jetson Nano System** section, you don't need to do the Update 2. You can skip this and proceed the **Set up MBot firmware** section now.
- If you have forked the [mbot_firmware](https://gitlab.eecs.umich.edu/rob550-f23/mbot_firmware) already, you need to update your forked code `mbot_firmware` with the upstream repo. The command line tool to do calibration and uploading firmware is fixed.

To update your forked `mbot_firmware`, firstly commit and push what you have locally to sync with the cloud, then click that `Update fork` button on the [upstream repo](https://gitlab.eecs.umich.edu/rob550-f23/mbot_firmware), then GitLab would automatically fetch and merge everything for you on the remote side. 

To pull the updated code locally:
1. Run “git fetch”  - to fetch what’s up new just to ensure we are at the right place
2. Run “git status” - git would tell you that your branch is behind
3. Run “git pull” - this time git will pull the code for you, and the insertions are happen locally
4. Now you are on sync! Remember to reboot the robot.

After finish these 2 updates, you can proceed `Calibrate the MBot and flash the firmware - Via the command-line tool`.