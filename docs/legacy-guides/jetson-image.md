---
layout: default
title: Jetson Image
parent: Legacy Guides
nav_order: 1
last_modified_at: 2024-01-13 14:37:48 -0500
---

> Here is a guide on how to create a customized Jetson image specifically for ROB550 botlab use.

{: .note}
This document is derived from Tom Gao's [notes](https://docs.google.com/document/d/1DL1buhZbwojC9O3xb2TjWVmkTUn2K0N1wiEUgWrj55I/edit#heading=h.dkx7mdu5b7jr)—shout out to him. However, it is updated but never tested as of the date this document was modified.

### Contents
* TOC
{:toc}

## 1. Flash the image

Download the Jetson Nano Ubuntu 20.04 bare image from [Q-engineering](https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html). We chose cutomized Ubuntu 20 over the official imagen from the NVIDIA website because the official one comes with Ubuntu 18, which is too outdated for our use.

Flash the downloaded image to your SD card. 

Boot up the Jetson then move to the next step.


## 2. Install latest kernel then disable updates
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


## 3. Dependencies and services
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


## 4. NoMachine
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


## 5. Customize the system
Current features:
1. Headless NoMachine remote login should be set at this point
2. MWireless Connection is not set, user needs to configure it themselves
3. No SSH Keys generated and botlab's repos are not cloned


### Making a Windows/MacOS accessible partition for `Mbot_config`
Move mmcblk0p1 128MiB to the right in GParted, then create fat32 partition in this empty space.
```bash
$ sudo mkdir /boot/firmware/
$ sudo nano /etc/fstab
```

Make `/dev/mmcblk0p15` (aka the last partition we created) mount at mount point /boot/firmware with type vfat (not fat32!).
Should look like the image below:
<a class="image-link" href="/assets/images/staff-guide/partition.png">
    <img src="/assets/images/staff-guide/partition.png" alt="" style="max-width:400px;"/>
</a>

Save and reboot.

### Add MBot as a user and grant sudo privileges
```bash
$ sudo adduser mbot
	#Put the default mbot password.
$ sudo adduser mbot sudo
```

### Set up auto login in gdm3
``` bash
$ sudo nano /etc/gdm3/custom.conf
```
Change to AutomaticLogin = mbot

### Set up nomachine for mbot user
```bash
$ sudo nano /usr/NX/etc/server.cfg
```
Edit owner:
DisplayOwner “mbot”

### Housekeeping
- Sidebar: files, firefox, terminal, settings
- Clean up desktop - *.desktop files moved to Documents/
- Clean out ~/ folders (keeping desktop documents downloads pictures)
- Put block M as background on both accounts
- Put SecureW2 setup shell in mbot/Downloads
- Shutdown and remove card. Put card in a Ubuntu machine.
    - Do not do this in a shared drive inside a Virtual Machine (e.g. UTM or Virtualbox). copy it to inside the VM and then do it. Symptom: e2fsck says loop device doesn’t have a valid superblock or something. https://forums.raspberrypi.com/viewtopic.php?t=18648


### Shrinking image
https://softwarebakery.com//shrinking-images-on-linux
In particular, comment http://disq.us/p/26jxmud.

It is wise to make a full dd copy of the image so far to avoid losing work!

### Resizing
You can either resize an SD card already flashed with the system image, or do it on a .img file on disk of a Linux system. The former can be done within a GUI (GParted) and is mostly self-explanatory and automatic, but can take additional time to flash if not already flashed. The latter is more risky, and you should use caution and take backup measures.

### Latest Note
Using GParted on Loopback:
It seems we can both get GParted support and also avoid flashing the card onto a physical medium. This supposedly launches GParted on a disk image and allows you to manipulate the disk partitions via GParted. This probably is the best method by far.
References: 
https://quorten.github.io/quorten-blog1/blog/2020/09/19/gparted-loop-dev 
```bash
$ sudo losetup -P /dev/loop0 disk1.img
$ sudo gparted /dev/loop0
```

Manual: Truncate empty space and repair GPT Entry
```bash
$ sudo fdisk -l <path-to-img-file>
```
Note the ending partition block (should be 5xxxxxxx for 512B sectors -> ~= 25 GB)
ADD 1+33 = 34 blocks for the backup table and header (or add more for good measure) and multiply by 512 like the guide suggests:
```bash
$ truncate --size=$[(5xxxxxxx+34)*512] <path-to-img-file>
```
The image has a broken GPT fs at the moment because we truncated the backup GPT table (at the end of the disk originally). This is normal, and we can use GDisk to repair this:
```bash
$ gdisk
```
Enter path to the image file again, do expert menu (x) -> rebuild backup GPT table (e) -> write (w).


Now, try loading the image to check integrity:
```bash
$ sudo modprobe loop
$ sudo losetup -f //Gives a usable /dev/loopxx device
$ sudo losetup /dev/loopxx <path-to-img-file>
$ sudo partprobe /dev/loopxx
	# Should have no errors here
$ sudo gparted /dev/loopxx
	#Should have the same partition layout sans empty space.
$ sudo losetup -d /dev/loopxx
```
Now we have a much smaller image.


{: .note}
Setup is done here - Students will start by connecting wifi, creating SSH key, downloading the codebase, etc.
