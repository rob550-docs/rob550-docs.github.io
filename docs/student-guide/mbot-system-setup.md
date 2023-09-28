---
layout: default
title: MBot System Setup
parent: Student Guide
nav_order: 2
last_modified_at: 2023-09-28 13:37:48 -0500
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

{: .highlight }
Now you have completed all the setup for Jetson! <br>At this point, the robot should publish its IP to the registry each time it turns on. The IP might change occasionally. You can now use VSCode, SSH, or NoMachine to interface with the MBot by using the IP it reports to the registry. 


## Calibrating and Flashing the MBot
1. Download the [firmware code](https://github.com/MBot-Project-Development/mbot_firmware) and [lcm base](https://github.com/MBot-Project-Development/mbot_lcm_base/tree/main)
    - Recommend to use VSCode remote extenstion + GitHub CLI 
2. Modify the code to compile the binary files
    1. Go to `mbot_firmware/src/mbot.h` line 34, change the drive to be differential 
        ```c
        //Define drive type of this robot. See mbot_params.h.
        //#define MBOT_DRIVE_TYPE OMNI_120_DRIVE
        #define MBOT_DRIVE_TYPE DIFFERENTIAL_DRIVE
        ```

    2. Install lcm related stuff
        ```bash
        $ cd ~/mbot_lcm_base
        $ ./scripts/install.sh
        ```

    3. Run the setup script
        ```bash
        $ cd ~/mbot_firmware
        $ ./setup.sh
        ```
        - If doesn't work, uncomment `git checkout master` in `setup.sh` and then run it again

    4. Build firmware
        ```bash
        $ cd ~/mbot_firmware
        $ mkdir build
        $ cd build
        $ cmake ..
        $ make
        ```

    5. You can find the `.uf2` under `/build`
        - The calibration script, `mbot_firmware/build/tests/mbot_calibrate_classic.uf2`
        - The MBot firmware, `mbot_firmware/build/src/mbot.uf2`

3. Calibrate the MBot
    
    In this step, we are going to use NoMachine to access to the MBot. 
    We will flash the calibration script onto the Pico to calibrate it before we flash it.

    The calibration script `mbot_calibrate_classic.uf2` detects the motor and encoder polarity and then calibrates the motor coefficients. The robot will move around for this step so you will need clear space on the floor (preferably on the same type of surface that you plan to use the robots on).

    1. Close the remote connection on VSCode and then connect the robot using NoMachine.
    2. Unplug the Robotics Control Board by disconnecting the barrel plug from the battery (leave the USB that powers the Jetson plugged in). Also unplug the USB that connects the Pico to the Jetson.

        <a class="image-link" href="/assets/images/system-setup/flash-prep.png">
        <img src="/assets/images/system-setup/flash-prep.png" alt=" " style="max-width:300px;"/>
        </a>

    3. To put the Pico in flashing mode, hold down the BOOTSEL button on the Pico board (it’s near the USB port). With the button held down, plug the Pico’s Type C cord back. Then release the button. The Pico should now show up as a device in NoMachine.

        <a class="image-link" href="/assets/images/system-setup/bootsel.png">
        <img src="/assets/images/system-setup/bootsel.png" alt=" " style="max-width:270px;"/>
        </a>
        <a class="image-link" href="/assets/images/system-setup/pico-nomachine.png">
        <img src="/assets/images/system-setup/pico-nomachine.png" alt=" " style="max-width:300px;"/>
        </a>

    4. Plug the barrel plug that powers the Robotics Control Board back into the battery.
    5. Place the MBot on the floor in a spot with at least 2 feet of clear space all around the robot.
    6. Open the Pico device folder in NoMachine. Drag and drop the script `mbot_calibrate_classic.uf2` into the folder. The Pico will reboot automatically, and will then run its calibration routine. **Don’t touch the robot while it does this procedure.**


        <iframe width="560" height="315" src="https://www.youtube.com/embed/PLdOf24KXX0?si=x7MH2hQUU5CnSrA4" title="YouTube video player" frameborder="10" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

4. Flash the MBot Firmware onto the Pico. 

    The calibration script will have saved parameters onto the Pico’s memory. We can now flash the firmware that will run on the Pico during operation. We will be repeating the flashing procedure.

    1. Repeat steps 3.1-3.3 from the calibration instructions to put the Pico into flashing mode.
    2. Open the Pico device folder in NoMachine. Drag and drop the script `mbot.uf2` into the folder. The Pico will reboot automatically.

## Install the MBot Code

This step will pull all the code utilities for the MBot Web App, SLAM, sensor drivers, and communication with the Robotics Control Board.

1. Clone the necessary repos
    - [RP Lidar Driver](https://github.com/MBot-Project-Development/rplidar_lcm_driver)
    - [MBot Autonomy](https://github.com/MBot-Project-Development/mbot_autonomy)
    - [MBot Bridge](https://github.com/MBot-Project-Development/mbot_bridge)

2. Install the MBot Web App
    1. Download the latest web app release and unpack it
    ```bash
    $ wget https://github.com/MBot-Project-Development/mbot_web_app/releases/download/v1.1.0/mbot_web_app-v1.1.0.tar.gz
    $ tar -xvzf mbot_web_app-v1.1.0.tar.gz
    ```
    2. Install the web app dependencies
    ```bash
    $ cd mbot_web_app-v1.1.0/
    $ ./install_nginx.sh
    $ ./install_python_deps.sh
    ```

    3. Build and install the app
    ```bash
    $ ./deploy_app.sh --no-rebuild
    ```
        - It’s now safe to delete the folder `mbot_web_app-v1.1.0/` and the tar file `mbot_web_app-v1.1.0.tar.gz.`

    {: .highlight }
    The web app should now be available!

    {: .note }
    You can use the web app by going to your browser and typing in the robot’s IP address. <br>
    If the firmware is flashed and the serial server is running, you should be able to drive the robot through the webapp. Toggle drive mode on then use the keys WSQE to drive the robot.


3. Install the RPLidar driver
```bash
$ cd ~/rplidar_lcm_driver/
$ ./scripts/install.sh
```
4. Install the MBot Autonomy code
```bash
$ cd ~/mbot_autonomy/
$ ./scripts/install.sh
```
- The autonomy code includes SLAM and a motion controller program.

5. Install the MBot Bridge and API 

    Firstly, go to `scripts/install.sh` and change all the lines with `python` to `python3`, otherwise it gives error:
    ```
    Traceback (most recent call last):
        File "setup.py", line 5, in <module>
        import setuptools
    ImportError: No module named setuptools
    ```
    
    Then run the following:
    ```bash
    $ cd ~/mbot_bridge/
    $ ./scripts/install.sh
    ```
    - The MBot Bridge includes a server that bridges student code with the MBot software, as well as APIs in C++ and Python.