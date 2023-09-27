---
layout: default
title: MBot System Setup
parent: Student Guide
nav_order: 2
last_modified_at: 2023-09-26 17:37:48 -0500
---


> This guide will walk you through the steps needed to setup the MBot Classic system. The guide is intended to be followed in order, do not jump back and forth.


### Contents
* TOC
{:toc}

## Setup Jetson Nano System

### What do you need
1. SD card for main storage
2. USB keyboard and mouse
3. Computer display (HDMI or DP)
4. Wifi dongle

### 1. Flash the image

1. Download the latest Jetson image from this [drive folder](https://drive.google.com/drive/folders/10ffPzANIETzGku317sOp2aiBFxPmd7Kn). 
    - We have the customized Ubuntu 20 over the official image from the NVIDIA website because the official one comes with Ubuntu 18, which is too outdated for our needs.
2. Download [Balena Etcher](https://etcher.balena.io/) then flash the OS image to your SD card.


### 2. Boot the Jetson Nano

1. Insert the SD card into your Jetson. The SD card slot is located on the side opposite the USB ports. (You might need to remove the camera mount to insert the card.)

    <a class="image-link" href="https://d29g4g2dyqv443.cloudfront.net/sites/default/files/akamai/embedded/images/jetsonNano/gettingStarted/Jetson_Nano-Getting_Started-Setup-Insert_microSD-B01.png">
    <img src="https://d29g4g2dyqv443.cloudfront.net/sites/default/files/akamai/embedded/images/jetsonNano/gettingStarted/Jetson_Nano-Getting_Started-Setup-Insert_microSD-B01.png" alt="Image from NVIDIA" style="max-width:300px;"/>
    </a>

2. Plug in mouse, keyboard, Wifi dongle, HDMI cable with external monitor.
3. Turn on the power bank and ensure that the power cables are connected as per the assembly guide.

If everything runs successfully, you will have an Ubuntu20 system ready for use.

{: .note }
Username: mbot <br>
Password: i<3robots!

### 3. Setup Wifi and update the system

1. If you are using home wifi, just connect it as you normally do
2. If you are using MWireless, run the following commands and enter your credentials when prompted
```bash
$ cd ~/Downloads
$ ./SecureW2_JoinNow.run     # this is wifi setup script
```

3. Run the well-known update, upgrade cycle
    ```bash
    $ sudo apt update
    $ sudo apt upgrade
    ```
 
### 4. VSCode Remote - SSH extension
In this step, we are going to establish remote access using VSCode on your laptop to connect to the Jetson remotely. After completing this step, we can download and modify files on the Jetson.

1. Get your Jetson's IP address 
    - Open a terminal on Jetson and run `ifconfig wlan0`, record your ip address, you will need it later    

    <a class="image-link" href="/assets/images/system-setup/ifconfig.png">
    <img src="/assets/images/system-setup/ifconfig.png" alt=" " style="max-width:400px;"/>
    </a>

2. On your laptop, install VSCode `Remote Development` extension pack

    <a class="image-link" href="/assets/images/system-setup/vscode_ssh1.png">
    <img src="/assets/images/system-setup/vscode_ssh1.png" alt=" " style="max-width:400px;"/>
    </a>

3. Add the SSH connection
    >Note that your laptop needs to connect to MWireless as well.
    
    <a class="image-link" href="/assets/images/system-setup/vscode_ssh2.png">
    <img src="/assets/images/system-setup/vscode_ssh2.png" alt=" " style="max-width:400px;"/>
    </a>

4. Input connection `ssh user_name@ip_address`

    Here your user_name should be `mbot`, ip_address is from step 4.1.

    <a class="image-link" href="/assets/images/system-setup/vscode_ssh3.png">
    <img src="/assets/images/system-setup/vscode_ssh3.png" alt=" " style="max-width:400px;"/>
    </a>

5. Select the default config file. Note that different operating systems may have different paths, but this isn't necessarily a problem. Here in the image, we select the one contains user name.

    <a class="image-link" href="/assets/images/system-setup/vscode_ssh4.png">
    <img src="/assets/images/system-setup/vscode_ssh4.png" alt=" " style="max-width:400px;"/>
    </a>

6. Navigate to the "Remote Explorer" tab and click the refresh button. You should see your Jetson's IP address listed under the SSH section, indicating that your connection has been set up. 
    - Click on "Connect in New Window" and enter the password `i<3robots!`. After this, your SSH session should be up and running.
    - To end the session, click on the tab on the bottom left corner labeled `SSH: xx.x.xxx.xx`. A pop-up menu with the `close remote connection` option will appear.

### 5. Install dependencies and services
At this point, you should be able to connect to the Jetson using VSCode extension.
Next, we are going to download and modify some files on Jetson.

1. Open a new Terminal in the VSCode remote session, then run:
```bash
$ git clone https://github.com/MBot-Project-Development/mbot_sys_utils.git
```

2. Run the following commands to execute install scripts
```bash
$ cd mbot_sys_utils/
# execute install scripts
$ sudo ./install_scripts/install_mbot_dependencies.sh
$ ./install_scripts/install_lcm.sh
```

3. Setup the MBot configuration
```bash
$ cd mbot_sys_utils/
# copy the config file to boot directory
$ sudo cp mbot_config.txt /boot/firmware/
# edit the config
$ sudo nano /boot/firmware/mbot_config.txt
```
- `mbot_hostname`: give the robot a unique hostname in this file, it should match the name written on the mbot.

4. Install udev rules and services 
```bash
# install udev rules
$ cd ~/mbot_sys_utils/udev_rules
$ ./install_rules.sh
# Install the services needed to start the networking and report the robot’s IP
$ cd ~/mbot_sys_utils/services
$ ./install_mbot_services.sh
```

5. Testing

    Restart the robot with `sudo reboot`, the Jetson will start to reboot and the connection will drop. You will need to reload the VSCode remote window. 
    - If everything is successful, the robot should publish its IP address to the [MBot IP registry](https://mbot-project-development.github.io/mbot_ip_registry/) as stipulated in the mbot_config.txt file.
    - If your hostname does not appear, you can execute the following steps for troubleshooting:
        ```bash
        $ cd ~/mbot_sys_utils
        $ ./systemctl_report.sh 
        ```
        The output will list the status of all the services, `mbot-start-network.service` and `mbot-start-network.service` both need to be `active`. If the status is "failed", use journalctl to see error logs of the service which might give a better idea of what's going wrong.
        ```bash
        $ sudo journalctl -u mbot-publish-info.service
        ```
        If the error message is still vague, ask the instructor for help.

### 6. Remote Desktop access - NoMachine
1. Download NoMachine to your laptop from the [official site](https://www.nomachine.com/).
    - NoMachine is a remote access software and it is pre-installed on the Jetson. 
 
2. Connect to Jetson using NoMachine
    - First, **unplug** your HDMI cable
    - Then, open NoMachine on your laptop, connect to Jetson as shown in the image below. You will need your IP address for this step, you can check IP address from [MBot IP registry](https://mbot-project-development.github.io/mbot_ip_registry/).

        <a class="image-link" href="/assets/images/system-setup/nomachine1.png">
        <img src="/assets/images/system-setup/nomachine1.png" alt=" " style="max-width:400px;"/>
        </a>
    - Finally, enter the username: `mbot`, password: `i<3robots!` to log in.

Now you have completed all the setup for Jetson!

## Calibrating and Flashing the MBot
1. Download the firmware code from this [github repo](https://github.com/MBot-Project-Development/mbot_firmware).
    - Recommend to use VSCode remote extenstion + GitHub CLI 
2. Go to `mbot_firmware/src/mbot.h` line 34, change the drive to be differential 

    ```c
    //Define drive type of this robot. See mbot_params.h.
    //#define MBOT_DRIVE_TYPE OMNI_120_DRIVE
    #define MBOT_DRIVE_TYPE DIFFERENTIAL_DRIVE
    ```
3. Compile  
    - The calibration script, `mbot_calibrate_classic.uf2`
    - The MBot firmware, `mbot.uf2`


## Install the MBot Code

