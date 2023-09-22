---
layout: default
title: MBot System Setup
parent: Getting Started
nav_order: 2
last_modified_at: 2023-09-22 17:37:48 -0500
---


> This guide will walk you through the steps needed to setup the MBot Classic system


### Contents
* TOC
{:toc}

## Setup Jetson Nano System

### What do you need
1. SD card for main storage
2. USB keyboard and mouse
3. Computer display (HDMI or DP)
4. Ethernet Cable
5. Wifi dongle

### 1. Flash the image

1. Download the Jetson Nano bare image from this [Github repo](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image). We chose this one over the official image from the NVIDIA website because the official one comes with Ubuntu 18, which is too outdated for our use.
2. Download [Balena Etcher](https://etcher.balena.io/) to flash the OS image to your SD card.
3. Flash the downloaded image to a blank SD card using Balena Etcher.

Now you have a SD card with Jetson Nano Ubuntu 20 image.

### 2. Boot the Jetson Nano

1. Insert the SD card into your Jetson. The SD card slot is located on the side opposite the USB ports.
    <a class="image-link" href="https://d29g4g2dyqv443.cloudfront.net/sites/default/files/akamai/embedded/images/jetsonNano/gettingStarted/Jetson_Nano-Getting_Started-Setup-Insert_microSD-B01.png">
    <img src="https://d29g4g2dyqv443.cloudfront.net/sites/default/files/akamai/embedded/images/jetsonNano/gettingStarted/Jetson_Nano-Getting_Started-Setup-Insert_microSD-B01.png" alt="Image from NVIDIA" style="max-width:300px;"/>
    </a>

2. Plug in mouse, keyboard, Ethernet cable (Or Wifi dongle), HDMI cable with external monitor.
3. Turn on the power bank and ensure that the power cables are connected as per the assembly guide.

If everything runs successfully, you will have an Ubuntu20 system ready for use.

{: .note }
Username: jetson <br>
Password: jetson

### 3. Setup Wifi

1. If you are using home wifi, just connect it as you normally do
2. If you are using MWireless, connect to the wifi following the University's 
[instruction](https://documentation.its.umich.edu/content/wifi-manually-configuring-your-ubuntu-linux-device-mwireless).


### 4. Configuring the Jetson

1. Run the well-known update, upgrade and autoremove cycle

    ```
    $ sudo apt update
    $ sudo apt upgrade
    $ sudo apt autoremove
    ```

2. Delete the directory /usr/share/vulkan/icd.d to prevent lavapipe warnings when using Jtop

    ```
    $ sudo rm -rf /usr/share/vulkan/icd.d
    ```

    While there are additional steps included in the Q-engineering post, they appear irrelevant to the current image provided in their GitHub repository at the time of writing this post. Therefore, I've omitted those steps for now. Should they become necessary in the future, I will update this post accordingly.

    Please note: If you experience any errors related to `nvidia-l4t-init` when running the upgrade command, a solution is provided in the Q-engineering [post](https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html).

    ```
    $ sudo apt-mark hold ‘nvidia-l4t-*’ 
    ```
    - If you wish to prevent upgrades during system updates for all packages that start with 'nvidia-l4t-', run this command.

{: .warning }
Do not install Chromium as it will interfere with the Snap installation. Use the preinstalled Morzilla Firefox.

### 5. Configuring remote desktop access - NoMachine

1. Install NoMachine server
    1. Go to the NoMachine for ARM [website](https://downloads.nomachine.com/linux/?id=30&distro=Arm) and download `ARMv8 DEB`.
    2. Navigate to the directory where the file located and run `$ sudo dpkg -i nomachinex.x.x_x_arm64.deb`

2. Make sure the jetson has auto login, this step is necessary to run headlessly using NoMachine
    - Edit `/etc/gdm3/custom.conf` if needed to have the file has the content below:
    
    ```
    $ sudo nano /etc/gdm3/custom.conf
    ```
    ```
    ...
    [daemon]
    AutomaticLoginEnable=true
    AutomaticLogin=jetson
    ...
    ```
3. Enable NetworkManager service
    ```
    $ sudo systemctl enable --now NetworkManager
    ```
4. Configure NoMachine
    1. Edit `/usr/NX/etc/server.cfg`
    ```
    $ sudo nano /usr/NX/etc/server.cfg
    ```
    2. Enable the following keys (remove the prepending #):
    ```    
    CreateDisplay 1
    DisplayOwner "jetson"
    DisplayGeometry 1600x900
    ```
        - Note that `ctrl + w` is search in nano, this is a very long file and it can really make you dizzy to scroll and look.
    3. reboot the jetson
    ```
    $ sudo reboot
    ```
    Right after shutting down (screen turned black), **unplug** HDMI cable and look for the blinking light on the Wifi dongle. 

    {: .warning }
    Do not plug in the HDMI if using the headless setup at any point when the jetson is powered. This could potentially interfere with the display settings and cause the display to freeze. Same goes for leaving it plugged in and then unplugging after it has already started the booting process.

5. Remote access

    If everthing works, you should see blinking light on the Wifi dongle. Now you can remotely access to the Jetson on your laptop using NoMachine.

    <a class="image-link" href="/assets/images/system-setup/nomachine-interface.png">
    <img src="/assets/images/system-setup/nomachine-interface.png" alt=" " style="max-width:400px;"/>
    </a>

### 6. Configuring SSH access - VSCode
1. Install the `Remote Development` extension pack

    <a class="image-link" href="/assets/images/system-setup/vscode_ssh1.png">
    <img src="/assets/images/system-setup/vscode_ssh1.png" alt=" " style="max-width:400px;"/>
    </a>

2. Add the SSH connection

    <a class="image-link" href="/assets/images/system-setup/vscode_ssh2.png">
    <img src="/assets/images/system-setup/vscode_ssh2.png" alt=" " style="max-width:400px;"/>
    </a>

3. Input connection `ssh hostname@ip_address`

    <a class="image-link" href="/assets/images/system-setup/vscode_ssh3.png">
    <img src="/assets/images/system-setup/vscode_ssh3.png" alt=" " style="max-width:400px;"/>
    </a>

4. Now you have a connection set up, you can connect to it under `remote explorer`


## Calibrating and Flashing the MBot


## Install the MBot Code


## Configuring the environment
1. Install `curl` and `venv` module for Python 3.8
    ```
    $ sudo apt install curl python3.8-venv
    ```


---

> Part of this doc is based on the [Q-engineering Blog](https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html) and Tom Gao's setup document.