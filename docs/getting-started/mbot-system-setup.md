---
layout: default
title: MBot System Setup
parent: Getting Started
nav_order: 2
last_modified_at: 2023-09-20 17:37:48 -0500
---


> This guide will walk you through the steps needed to setup the MBot Classic system
 
### Contents
* TOC
{:toc}

## Setup Jetson Nano System

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

2. Plug in mouse, keyboard, Ethernet cable, HDMI cable with external monitor.
3. Turn on the power bank and ensure that the power cables are connected as per the assembly guide.

If everything runs successfully, you will have an Ubuntu20 system ready for use.

{: .note }
Username: jetson <br>
Password: jetson


### 3. Configuring the Jetson

> The following configuration is based on the [Q-engineering Blog](https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html#:~:text=The%20Jetson%20Nano%20comes%20with,20.04%20on%20the%20Jetson%20Nano.) and Tom Gao's setup document.

1. Run the well-known update, upgrade and autoremove cycle

```
$ sudo apt update
$ sudo apt upgrade
$ sudo apt autoremove
```

<!-- Resume from 
the section: Starting Ubuntu 20.04 on your Nano. -->


