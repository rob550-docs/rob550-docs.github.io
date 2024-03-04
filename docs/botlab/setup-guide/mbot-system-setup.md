---
layout: default
title: MBot System Setup
parent: Setup Guide
grand_parent: Botlab
nav_order: 2
last_modified_at: 2024-03-04 16:03:48 -0500
---

> This guide will walk you through the steps needed to setup the MBot Classic system. The guide is intended to be followed in order, do not jump back and forth.

The following items are needed:
1. microSD card for main storage
2. SD adapter
3. USB keyboard and mouse


### Contents
- TOC
{:toc}

## Set up Jetson Nano System
### 1. Flash the image
1. Download the custom Jetson image `jetson-rob550_oct19.img.xz` from this [link](https://www.dropbox.com/scl/fi/yxhb7gynpxxj639clfbpt/jetson-rob550_oct19.img.xz?rlkey=fq3c2ltk3bkr0fr8vvd1i4u4h&dl=0).
    - We use a custom image with Ubuntu 20.04 instead of the official image from NVIDIA because the official uses Ubuntu 18.04, which is outdated.
2. Download [Balena Etcher](https://etcher.balena.io/) then flash the OS image to your SD card. Plug in the SD card to your laptop using SD card reader then following the steps in Balena Etcher

You now have an SD card with Ubuntu 20.04 flashed on it for the Jetson. Keep the card in your laptop for now and proceed to the next step.

### 2. Set up system utilities

If the flash succeeded, the SD card will have two partitions: a 134MB Volume formatted as fat32 and a 27GB Volume formated as ext4. When you insert the SD card in your laptop, it should mount the smaller fat32 partition.  Find the file `mbot_config.txt` on this volume and modify it as follows:
- Set `mbot_hostname` following this format: `mbot-<section>-<team#>-<unique_name>`
    - For example: if you are in the AM section team 6, and your unique_name is johndoe, you should name the robot as `mbot-AM-team6-johndoe`
- Enter your home Wi-Fi details for `new_wifi_ssid` and `new_wifi_password` if you intend to use it at home later.

### 3. Boot the Jetson Nano
1. Insert the SD card into your Jetson. The SD card slot is located on the side opposite the USB ports. 

    <a class="image-link" href="https://d29g4g2dyqv443.cloudfront.net/sites/default/files/akamai/embedded/images/jetsonNano/gettingStarted/Jetson_Nano-Getting_Started-Setup-Insert_microSD-B01.png">
    <img src="https://d29g4g2dyqv443.cloudfront.net/sites/default/files/akamai/embedded/images/jetsonNano/gettingStarted/Jetson_Nano-Getting_Started-Setup-Insert_microSD-B01.png" alt="Image from NVIDIA" style="max-width:300px;"/>
    </a>

2. Turn on the power bank and ensure that the power cables are connected as per the assembly guide.
3. If everything runs smoothly, the OLED screen on the side will light up after a minute or so, and the robot hostname you used in the second step should appear in the [MBot IP registry](https://gitlab.eecs.umich.edu/rob550-f23/mbot_ip_registry).
4. You can check the Jetson's IP address on the OLED screen or locate the related JSON file in the [MBot IP registry](https://gitlab.eecs.umich.edu/rob550-f23/mbot_ip_registry) data folder.

{: .note }
Every time the robot starts, an update to the IP JSON file is pushed to the registry. 

### 4. VSCode Remote - SSH extension
> In this step, we are going to establish remote access using the VSCode extension. After this setup, you will be able to access the Jetson remotely using your laptop.

1. On your laptop, install VSCode `Remote Development` extension pack

    <a class="image-link" href="/assets/images/botlab/system-setup/vscode_ssh1.png">
    <img src="/assets/images/botlab/system-setup/vscode_ssh1.png" alt=" " style="max-width:400px;"/>
    </a>

2. Add the SSH connection
    >Note that your laptop needs to connect to MWireless as well.
    
    <a class="image-link" href="/assets/images/botlab/system-setup/vscode_ssh2.png">
    <img src="/assets/images/botlab/system-setup/vscode_ssh2.png" alt=" " style="max-width:400px;"/>
    </a>

3. Input connection `ssh mbot@your_mbot_ip_address`

    <a class="image-link" href="/assets/images/botlab/system-setup/vscode_ssh3.png">
    <img src="/assets/images/botlab/system-setup/vscode_ssh3.png" alt=" " style="max-width:400px;"/>
    </a>

4. Select the default config file. Note that different operating systems may have different paths, but this isn't necessarily a problem. Here in the image, we select the one contains user name.

    <a class="image-link" href="/assets/images/botlab/system-setup/vscode_ssh4.png">
    <img src="/assets/images/botlab/system-setup/vscode_ssh4.png" alt=" " style="max-width:400px;"/>
    </a>

5. Navigate to the "Remote Explorer" tab and click the refresh button. You should see your Jetson's IP address listed under the SSH section, indicating that your connection has been set up. 
    - Click on "Connect in New Window" and enter the password `i<3robots!`. After this, your SSH session should be up and running.
    - To end the session, click on the tab on the bottom left corner labeled `SSH: xx.x.xxx.xx`. A pop-up menu with the `close remote connection` option will appear.


{: .highlight }
At this point, you should be able to connect to the Jetson using VSCode extension.

{: .note }
Username: mbot <br>
Password: i<3robots!

### 5. Remote Desktop access - NoMachine
> In this step, we are going to set up NoMachine access. Upon completion, you will be able to access the Jetson with a Desktop UI. 

1. Download NoMachine to your laptop from the [official site](https://www.nomachine.com/).
    - NoMachine is a remote access software and it is pre-installed on our customized Jetson. 
 
2. Connect to Jetson using NoMachine
    - Open NoMachine on your laptop, connect to Jetson as shown in the image below. You will need your IP address for this step.

        <a class="image-link" href="/assets/images/botlab/system-setup/nomachine1.png">
        <img src="/assets/images/botlab/system-setup/nomachine1.png" alt=" " style="max-width:400px;"/>
        </a>
    - Finally, enter the username: `mbot`, password: `i<3robots!` to log in.
    - Note: if the NoMachine desktop freezes, you can always restart the robot by turning the power off and then back on.

{: .highlight }
Now you have completed all the setup for Jetson!

## Update system utilities
The `mbot_sys_utils` has been updated after the OS image was generated, and we need to update the settings accordingly. This step is essential for the firmware set up later.

1. Pull the latest changes from the `mbot_sys_utils` repository:
```bash
$ cd ~/mbot_sys_utils
$ git pull
```
2. Append the last line of `mbot_config.txt` to the system configuration:
```bash
$ tail -n 1 mbot_config.txt | sudo tee -a /boot/firmware/mbot_config.txt > /dev/null 
```
To verify the update, execute the command below, and you should see the output "autostart=run" in your terminal. Ensure this output is present before proceeding to the next step.
```bash
$ tail -n 1 /boot/firmware/mbot_config.txt && echo ""
```
3. Install the services manually:
```bash
$ cd ~/mbot_sys_utils/services
$ ./install_mbot_services.sh
```
After installation, power off the device, then turn it back on and reconnect.

4. Ensure the updates are properly loaded by checking the log:
```bash
$ code /var/log/mbot/mbot_start_networking.log
```
At the bottom of this log file, if you see the message "Autostart is set to run", that means your update was successful.
```
===== 2023-10-24 22:09:19 =====
hostname set to 'mbot-0000-example'
Connected to active WiFi network 'exmaple'. Done.
Autostart is set to run 
```


## Set up MBot firmware
> In this session, we are going to work on setup of the Control Board.

### 1. Compile the firmware files
1. It is always a good practice to create a dedicated workspace when working on large-scale projects. Open a new Terminal in the VSCode remote session, then create a new folder called `mbot_ws`:
```bash
$ mkdir mbot_ws 
```

    Next, **fork** [mbot_firmware](https://gitlab.eecs.umich.edu/rob550-f23/mbot_firmware) and [mbot_lcm_base](https://gitlab.eecs.umich.edu/rob550-f23/mbot_lcm_base) to your group first, you will need to modify them for course assignment later, **then clone** your forked codebase to Jetson `/mbot_ws`.
        
2. Compile the firmware code to get .uf2 binary files
    1. Install lcm related stuff
        ```bash
        $ cd ~/mbot_ws/mbot_lcm_base
        $ ./scripts/install.sh
        ```
    2. Run the firmware setup script
        ```bash
        $ cd ~/mbot_ws/mbot_firmware
        $ ./setup.sh
        ```
    3. Build firmware
        ```bash
        $ cd ~/mbot_ws/mbot_firmware
        $ mkdir build
        $ cd build
        $ cmake ..
        $ make
        ```
    4. Now you will have 2 relevant `.uf2` files under `/build`
        - The calibration script, `mbot_firmware/build/tests/mbot_calibrate_classic.uf2`
        - The MBot firmware, `mbot_firmware/build/src/mbot.uf2`


### 2. Calibrate the MBot and flash the firmware
In this step, we are going to flash the calibration script onto the Pico to calibrate it before we flash the firmware. There are 2 options to proceed calibration routine:

**1. Via the command-line tool (Recommended)**

1. Place the MBot on the floor in a spot with at least 2 feet of clear space all around the robot, preferably on the same type of surface that you plan to use the robots on.
2. Run the following command, the Pico will reboot automatically, and will then run its calibration routine right away. Allow the Pico to finish its calibration routine without interference.

    {: .warning}
    Hold off on running this command until the robot is placed on the floor.

    ```bash
    $ cd ~/mbot_ws/mbot_firmware
    # upload the calibration scripts
    $ sudo ./upload.sh flash build/tests/mbot_calibrate_classic.uf2
    ```
Note that during the calibration routine, robot should turning in **counter clockwise** circle first then turning **clockwise**. If it is not executing in this order, you might have wrong motor polarity. Modify it in the `mbot_firmware/tests/mbot_calibrate_classic.c` to be either 1 or -1.
    ```
    #define MOT_LEFT_POL 1    
    #define MOT_RIGHT_POL 1
    ```

    Here is a video of expected routine:

    <iframe width="400" height="227" src="https://www.youtube.com/embed/iIghZzf8ZQY?si=w_ultg6PtCwJVr2_" title="YouTube video player" frameborder="2" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

The calibration script will have saved parameters onto the Pico’s memory. We can now flash the firmware that will run on the Pico during operation.

```bash
$ cd ~/mbot_ws/mbot_firmware
$ sudo ./upload.sh flash build/src/mbot.uf2
```

**2. Via BOOTSEL Mode**
1. Initiate a remote connection with VSCode.
2. Temporarily disconnect the Control Board by removing both the battery's barrel plug and USB-C while keeping the Jetson powered on. Ensure no cables are connected to the Control Board.
3. Enter the Pico bootloader mode (BOOTSEL mode) in the following order: press and hold the BOOTSEL button on the board; reconnect the USB-C to Pico (while holding down the BOOTSEL button); release the button; and finally, reconnect the power barrel plug.

    <a class="image-link" href="/assets/images/botlab/system-setup/bootsel.png">
    <img src="/assets/images/botlab/system-setup/bootsel.png" alt=" " style="max-width:270px;"/>
    </a>

4. Upload the calibration file by entering the following commands:
```bash
$ cd ~/mbot_ws/mbot_firmware 
$ sudo picotool load build/tests/mbot_calibrate_classic.uf2
```

5. Place the MBot on the floor in a spot with at least 2 feet of clear space all around the robot.
```bash
$ sudo picotool reboot
```
6. The Pico will reboot automatically, and will then run its calibration routine. Allow the Pico to finish its calibration routine without interference.

The calibration script will have saved parameters onto the Pico’s memory. We can now flash the firmware that will run on the Pico during operation.

- Repeat the step 2 and 3 above to put the Pico to bootloader mode, but this time run
```bash
$ cd ~/mbot_ws/mbot_firmware
$ sudo picotool load build/src/mbot.uf2
$ sudo picotool reboot
```

### 3. Using Minicom
Here we introduce you the tool Minicom. It is a program designed for serial communication that connects devices to a Linux PC via serial ports, we will use Minicom to read the pico printouts from the Jetson module.

- After flashing the firmware to the Pico, run the following command to start minicom
    ```bash
    $ minicom -D /dev/mbot_tty -b 115200
    ```
    - `-D` indicates the serial port device, and `-b` sets the communication speed or baud rate.
    - If the minicom command doesn't work, run this: `ls /dev | grep mbot`
    ```bash
    $ mbot@mbot-0018-shaw:/dev$ ls /dev | grep mbot
    mbot_lcm
    mbot_tty
    ```
    If you do not see the 2 outputs above, unplug the USB which connect Jetson and Pico, then plug back in.
- To exit Minicom, press `CTRL-A` to get to command mode, then press `X` to quit.

**Successful Firmware Flashing:** After flashing the firmware successfully, Minicom will display your encoder counts, IMU values, and more.
Manually turning the wheel will update the encoder counts in the Minicom terminal.

**Unsuccessful Firmware Flashing:** If the firmware doesn't flash correctly, repeat the calibration and firmware flashing steps.
Open a second terminal window with Minicom to monitor its outputs for troubleshooting.


### Install the rest of the MBot Code

1. Clone the necessary repos to your Jetson under folder `mbot_ws`
    - **Clone** [RP Lidar Driver](https://gitlab.eecs.umich.edu/rob550-f23/rplidar_lcm_driver) and [MBot Bridge](https://gitlab.eecs.umich.edu/rob550-f23/mbot_bridge)
    - **Fork** [MBot Autonomy](https://gitlab.eecs.umich.edu/rob550-f23/mbot_autonomy)to your group and **then clone** the forked code to Jetson

2. Install the MBot Web App
    1. Download the latest web app release and unpack it
    ```bash
    $ cd mbot_ws
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
    The web app should now be available! You don't have to re-run any of these steps; the web app will automatically start when you start the robot.

    {: .note }
    You can use the web app by going to your browser and typing in the robot’s IP address. <br>
    If the firmware is flashed and the serial server is running, you should be able to drive the robot through the webapp. Toggle drive mode on then use the keys WSQE to drive the robot.


3. Install the RPLidar driver
```bash
$ cd ~/mbot_ws/rplidar_lcm_driver/
$ ./scripts/install.sh
```
4. Install the MBot Autonomy code
```bash
$ cd ~/mbot_ws/mbot_autonomy/
$ ./scripts/install.sh
```
- The autonomy code includes SLAM and a motion controller program.

5. Install the MBot Bridge and API 
    ```bash
    $ cd ~/mbot_ws/mbot_bridge/
    $ ./scripts/install.sh
    ```
    - The MBot Bridge includes a server that bridges student code with the MBot software, as well as APIs in C++ and Python.
