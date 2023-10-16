---
layout: default
title: MBot System Setup
parent: Student Guide
nav_order: 2
last_modified_at: 2023-10-09 17:37:48 -0500
---


> This guide will walk you through the steps needed to setup the MBot Classic system. The guide is intended to be followed in order, do not jump back and forth.

The following items are needed:
1. microSD card for main storage
2. SD adapter
3. USB keyboard and mouse
4. Computer display (HDMI or DP)


### Contents
- TOC
{:toc}


## Setup Jetson Nano System

### 1. Flash the image

1. Download the latest Jetson image from this [drive folder](https://drive.google.com/drive/folders/10ffPzANIETzGku317sOp2aiBFxPmd7Kn) to your laptop.
    - There are multiple images in the folder, download `mbot-jetson-jul31.img.xz`.
    - We use the customized Ubuntu 20 instead of the official image from the NVIDIA website because the official one comes with Ubuntu 18, which is too outdated for our needs.
2. Download [Balena Etcher](https://etcher.balena.io/) then flash the OS image to your SD card.
    1. Open the Balena Etcher
    2. Plug in the SD card to your laptop using SD card reader
    3. Following the steps on Balena Etcher

Now, you have an SD card with the Ubuntu system flashed on it. We will boot up the Jetson using this SD card.

### 2. Boot the Jetson Nano

1. Insert the SD card into your Jetson. The SD card slot is located on the side opposite the USB ports. 

    <a class="image-link" href="https://d29g4g2dyqv443.cloudfront.net/sites/default/files/akamai/embedded/images/jetsonNano/gettingStarted/Jetson_Nano-Getting_Started-Setup-Insert_microSD-B01.png">
    <img src="https://d29g4g2dyqv443.cloudfront.net/sites/default/files/akamai/embedded/images/jetsonNano/gettingStarted/Jetson_Nano-Getting_Started-Setup-Insert_microSD-B01.png" alt="Image from NVIDIA" style="max-width:300px;"/>
    </a>

2. Plug in mouse, keyboard, HDMI cable with external monitor.
3. Turn on the power bank and ensure that the power cables are connected as per the assembly guide.

If everything runs successfully, you will have an Ubuntu 20 system ready for use.

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
- If you have message saying "...Configured Network is however not in range", it is likely that you have entered the wrong password, end the process and re-run the command.

3. Run the well-known update, upgrade cycle
    ```bash
    $ sudo apt update
    $ sudo apt upgrade
    ```
 
### 4. VSCode Remote - SSH extension
> In this step, we are going to establish remote access using the VSCode extension. After this setup, you will be able to access the Jetson remotely using your laptop.

1. Get your Jetson's IP address 

    Open a terminal on Jetson and run `ifconfig wlan0`, record your ip address, you will need it later. You can either write it down on paper, or use [wormhole tool](how-to-guide.html#how-transfer-file-from-mbot-to-your-laptop---wormhole) to send it in a txt file to your laptop.    

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


{: .highlight }
At this point, you should be able to connect to the Jetson using VSCode extension.

### 5. Install dependencies and services
> In this step, we are going to access the Jetson remotely, edit config files and eventually gives your robot a unique name.

{: .text-red-300 .fs-6}
**Under Editing...**


1. Open a new Terminal in the VSCode remote session, then run:
```bash
$ git clone https://gitlab.eecs.umich.edu/rob550-f23/mbot_sys_utils.git
```

2. Run the following commands to install system utilities 
```bash
$ cd mbot_sys_utils/
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
    - If the setup was successful, the robot should publish its IP address to the [MBot IP registry](https://gitlab.eecs.umich.edu/rob550-f23/mbot_ip_registry), you can find `your_hostname.json` file under `/data` folder and check your robot's IP address there.
    - If your hostname does not appear, you can execute the following steps for troubleshooting:
        ```bash
        $ cd ~/mbot_sys_utils
        $ ./systemctl_report.sh 
        ```
        The output will list the status of all the services, `mbot-start-network.service` and `mbot-start-network.service` both need to be `active`. If the status is "failed", ask the instructor for help.

    {: .note }
    Every time the robot starts, an update to the IP JSON file is pushed to the registry. This is useful when running headless. Without a monitor, the IP registry is one way to check your current IP since it might change randomly. You can also check your IP address on the OLED screen on the side of the robot which we will set up later.

### 6. Remote Desktop access - NoMachine
> In this step, we are going to set up NoMachine access. Upon completion, you will be able to access the Desktop UI. This is unlike the VSCode extension, which allows access to the Jetson only over Terminal.

1. Download NoMachine to your laptop from the [official site](https://www.nomachine.com/).
    - NoMachine is a remote access software and it is pre-installed on the Jetson. 
 
2. Connect to Jetson using NoMachine
    - First, **unplug** your HDMI cable if it is still connected
    - Then, open NoMachine on your laptop, connect to Jetson as shown in the image below. You will need your IP address for this step, you can check IP address from [MBot IP registry](https://gitlab.eecs.umich.edu/rob550-f23/mbot_ip_registry).

        <a class="image-link" href="/assets/images/system-setup/nomachine1.png">
        <img src="/assets/images/system-setup/nomachine1.png" alt=" " style="max-width:400px;"/>
        </a>
    - Finally, enter the username: `mbot`, password: `i<3robots!` to log in.
    - Note: if the NoMachine desktop freezes, you can always restart the robot by turning the power off and then back on.

{: .highlight }
Now you have completed all the setup for Jetson!


## Calibrating and Flashing the MBot
> In this session, we are going to work on setup of the Control Board.

1. Download code base
    1. **Fork** [mbot_firmware](https://gitlab.eecs.umich.edu/rob550-f23/mbot_firmware) to your group first, you will need to modify the firmware code for course assignment later, **then clone** your forked firmware codebase to Jetson
    2. **Clone** [mbot_lcm_base](https://gitlab.eecs.umich.edu/rob550-f23/mbot_lcm_base) to Jetson
    
    Recommend to use VSCode remote extenstion + Clone with HTTPS

2. Compile the firmware code to get .uf2 binary files
    1. Install lcm related stuff
        ```bash
        $ cd ~/mbot_lcm_base
        $ ./scripts/install.sh
        ```
    2. Run the firmware setup script
        ```bash
        $ cd ~/mbot_firmware
        $ ./setup.sh
        ```
    3. Build firmware
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

3. Calibrate the MBot and flash the MBot Firmware onto the Pico. 
    - In this step, we are going to flash the calibration script onto the Pico to calibrate it before we flash the firmware.
    - The calibration script `mbot_calibrate_classic.uf2` detects the motor and encoder polarity and then calibrates the motor coefficients. The robot will move around for this step so you will need clear space on the floor (preferably on the same type of surface that you plan to use the robots on).

    For this step, there are 2 options to proceed:

    **1. Using VSCode Extention + Picotool**
    1. Establish VSCode’s remote connection 
    2. Installing [picotool](https://github.com/raspberrypi/picotool)
    ```bash
    $ wget https://github.com/raspberrypi/picotool/archive/refs/tags/1.1.1.zip
    $ unzip 1.1.1.zip
    $ cd picotool-1.1.1
    $ mkdir build && cd build
    $ export PICO_SDK_PATH=~/mbot_firmware/lib/pico-sdk
    $ cmake ..
    $ make
    $ sudo make install
    ```
    3. Disconnect the battery's barrel plug and the USB C from the Control Board while leaving the Jetson power on. Essentially, make sure there are no cables connected to the Control Board.
    4. To put the Pico in BOOTSEL mode, or say bootloader mode: Press and hold the `BOOTSEL` button on the board. While holding the `BOOTSEL` button down, connect the USB C back to Pico. Then release the button. 
    5. Load the calibration file
    ```bash
    $ cd mbot_firmware 
    $ picotool load build/tests/mbot_calibrate_classic.uf2
    ```
    6. Place the MBot on the floor in a spot with at least 2 feet of clear space all around the robot.
    ```bash
    $ picotool reboot
    ```
    7. The Pico will then run its calibration routine. **Don’t touch the robot while it does this procedure.**

    The calibration script will have saved parameters onto the Pico’s memory. We can now flash the firmware that will run on the Pico during operation.

    - Repeat the step c and d above, but this time run
    ```bash
    $ cd mbot_firmware 
    $ picotool load build/src/mbot.uf2
    $ picotool reboot
    ```


    **2. Using NoMachine**  
    1. Terminate VSCode's remote connection and establish a connection through NoMachine.
    2. Disconnect the battery's barrel plug and the USB C from the Control Board while leaving the Jetson power on. Essentially, make sure there are no cables connected to the Control Board.
    3. To put the Pico in BOOTSEL mode, hold down the `BOOTSEL` button on the Pico board. With the button held down, plug the Pico’s Type C cord back. Then release the button. 
        - While in this mode, the device will mount as a mass storage peripheral (like a flash drive). During this time, you can load programs onto it by dragging and dropping .uf2 files onto the device.
        
        <a class="image-link" href="/assets/images/system-setup/bootsel.png">
        <img src="/assets/images/system-setup/bootsel.png" alt=" " style="max-width:270px;"/>
        </a>
        <a class="image-link" href="/assets/images/system-setup/pico-nomachine.png">
        <img src="/assets/images/system-setup/pico-nomachine.png" alt=" " style="max-width:300px;"/>
        </a>

    4. Plug the barrel plug back into Robotics Control Board.
    5. Place the MBot on the floor in a spot with at least 2 feet of clear space all around the robot.
    6. Open the Pico device folder in NoMachine. Drag and drop the script `mbot_calibrate_classic.uf2` into the folder. The Pico will reboot automatically, and will then run its calibration routine. **Don’t touch the robot while it does this procedure.**

        <iframe width="560" height="315" src="https://www.youtube.com/embed/PLdOf24KXX0?si=x7MH2hQUU5CnSrA4" title="YouTube video player" frameborder="10" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

    The calibration script will have saved parameters onto the Pico’s memory. We can now flash the firmware that will run on the Pico during operation.

    1. Repeat steps 1-3 from the calibration instructions to put the Pico into flashing mode.
    2. Open the Pico device folder in NoMachine. Drag and drop the script `mbot.uf2` into the folder. The Pico will reboot automatically.

## Install the MBot Code

1. Clone the necessary repos to your Jetson 
    - **Clone** [RP Lidar Driver](https://gitlab.eecs.umich.edu/rob550-f23/rplidar_lcm_driver)
    - **Clone** [MBot Bridge](https://github.com/MBot-Project-Development/mbot_bridge)
    - **Fork** [MBot Autonomy]( ) to your group 
    and **then clone** the forked code to Jetson

    Recommend to use VSCode remote extenstion + Clone with HTTPS

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
    ```bash
    $ cd ~/mbot_bridge/
    $ ./scripts/install.sh
    ```
    - The MBot Bridge includes a server that bridges student code with the MBot software, as well as APIs in C++ and Python.