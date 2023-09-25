---
layout: default
title: Jetson Image
parent: Staff Guide
nav_order: 1
last_modified_at: 2023-09-25 14:37:48 -0500
---

> Here is a guide on how to create a Jetson image. This document is derived from Tom's [notes](https://docs.google.com/document/d/1DL1buhZbwojC9O3xb2TjWVmkTUn2K0N1wiEUgWrj55I/edit#heading=h.dkx7mdu5b7jr).

### Contents
* TOC
{:toc}

### 1. Flash the image

Download the Jetson Nano bare image `Ubuntu20.04_Jetson_Fresh.img.xz` from this [drive folder](https://drive.google.com/drive/folders/10ffPzANIETzGku317sOp2aiBFxPmd7Kn). We chose cutomized Ubuntu20 over the official image from the NVIDIA website because the official one comes with Ubuntu 18, which is too outdated for our use.

Flash the image to your SD card. 

Boot up the Jetson then move to the next step.


### 2. Install latest kernel then disable updates
Run the well-known update, upgrade cycle
```bash    
$ sudo apt update
$ sudo apt upgrade    
```


If you experience any errors related to `nvidia-l4t-init` when running the upgrade command, a solution is provided in the Q-engineering [post](https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html), which also provided below:

```bash  
$ sudo apt --fix-broken install
$ sudo dpkg -i --force-overwrite <...path to l4t .deb file>
$ sudo apt upgrade 
```
```bash  
$ sudo apt-mark hold ‘nvidia-l4t-*’ 
```
- This is to prevent further upgrades during system updates for all packages that start with 'nvidia-l4t-', run this command.


### 3. Dependencies and services
1. Install higher version of CMake
    - Download [this file](https://github.com/Kitware/CMake/releases/download/v3.27.0/cmake-3.27.0-linux-aarch64.sh) to `/opt`
    - Then run the following commands
```bash  
$ cd /opt
$ chmod +x cmake-<....>.sh
$ sudo ./cmake-<....>.sh
# Press y twice (for licensing etc)
$ sudo ln -s /opt/cmake-<....>/bin/* /usr/local/bin
$ cmake --version # checks that 3.27 installed successfully
```

2. Install curl and python venv module
```bash  
$ sudo apt install curl python3.8-venv
```

3. Enable NetworkManager service
```bash  
$ sudo systemctl enable --now NetworkManager
$ sudo systemctl status NetworkManager # to check status
```

4. Disable disgusting looking nvidia logo (still leave the power menu up)
```bash  
$ sudo rm /usr/share/nvpmodel_indicator/nv_logo.svg
```


### 4. NoMachine
1. Install NoMachine server
    1. Go to the NoMachine for ARM [website](https://downloads.nomachine.com/linux/?id=30&distro=Arm) and download `ARMv8 DEB`.
    2. Navigate to the directory where the file located and run `$ sudo dpkg -i nomachinex.x.x_x_arm64.deb`
2. Make sure the jetson has auto login, this step is necessary to run headlessly using NoMachine
    - Edit `/etc/gdm3/custom.conf` if needed to have the file has the content below:
    
    ```bash  
    $ sudo nano /etc/gdm3/custom.conf
    ```
    ```conf
    ...
    [daemon]
    AutomaticLoginEnable=true
    AutomaticLogin=jetson
    ...
    ```

3. Configure NoMachine (create new display when HDMI unplugged)
    1. Edit `/usr/NX/etc/server.cfg`
    ```bash
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
    ```bash
    $ sudo reboot
    ```
    Right after shutting down (screen turned black), **unplug** HDMI cable and look for the blinking light on the Wifi dongle. 

    {: .warning }
    Do not plug in the HDMI if using the headless setup at any point when the jetson is powered. This could potentially interfere with the display settings and cause the display to freeze. Same goes for leaving it plugged in and then unplugging after it has already started the booting process.


    ---

    Under Construction...