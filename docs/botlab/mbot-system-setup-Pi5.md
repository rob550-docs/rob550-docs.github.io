---
layout: default
title: MBot System Setup
parent: Botlab
nav_order: 2
last_modified_at: 2025-03-11 12:59:00 -0500
---

{: .important}
This guide is for MBot Classic using **Raspberry Pi 5**!

> This guide will walk you through the steps needed to setup the MBot Classic system. The guide is intended to be followed in order, do not jump back and forth.

The following items are needed:
1. microSD card for main storage
2. SD adapter
3. A laptop can read and write SD card


### Contents
- TOC
{:toc}

**Important:** Do not power on the MBot before following the system setup guide. The SD card you receive might not be empty or could have the wrong operating system installed. Powering on the device before checking could damage the Raspberry Pi.
{: .text-red-200}

If you've already powered on the Raspberry Pi and notice the LED light has "4 long flashes followed by 5 short flashes", it indicates a [Fatal Firmware Error](https://www.raspberrypi.com/documentation/computers/configuration.html#led-warning-flash-codes). You can find the solution in the troubleshooting guide [here](/docs/botlab/how-to-guide/pi-troubleshooting).
- The LED light, labeled "STAT," is located next to the SD slot.

## Set up RPi 5 System
### 1. Flash the image
1. Download the custom Pi5 image `2024-11-20-mbot-base-bookworm.img.gz` from this [link](https://www.dropbox.com/scl/fi/psz70s9ja5syyhhk82dv3/2024-11-20-mbot-base-bookworm.img.gz?rlkey=71fy1nf2hqf6s8fq2r81za9lk&st=qxs08er4&dl=0) to your laptop. We use a custom image with RPiOS based on Debian 12 Bookworm
2. Download [Balena Etcher](https://etcher.balena.io/) to your laptop, it is a tool to flash the OS image to the SD card. Plug in the SD card to your laptop using SD card reader then following the steps in Balena Etcher.

You now have an SD card with the OS image flashed on it for the Pi5. Keep the card in your laptop for now and proceed to the next step.

{: .warning }
If you do the flashing on a Windows computer, you may see many file explorer windows and error messages pop up when you insert the SD card and when you finish flashing. Those are expected, and you can safely close the file explorer windows and dismiss the error messages. However, if Windows asks you to format the SD card through a popup dialog box, close the message through the "Cancel" button and **do not** click the "Format Disk" button.


### 2. Set up system utilities

If the flash succeeded, the SD card will have two partitions: a 134MB Volume formatted as fat32 and a 16GB Volume formated as ext4. Open the file explorer on your laptop, it should mount the smaller fat32 partition named "bootfs".

Find the file `mbot_config.txt` on this volume and modify it as follows:
- Set `mbot_hostname` following this format: `mbot-<section>-<team#>-<unique_name>`
    - For example: if you are in the AM section team 6, and your unique_name is johndoe, you should name the robot as `mbot-AM-team6-johndoe`
- Enter your home Wi-Fi details for `new_wifi_ssid` and `new_wifi_password` if you intend to use it at home later.
- Leave all other variables unchanged.

Then save the file. Now you can eject the SD card.

### 3. Boot the Pi5
1. Insert the SD card into your Pi5. The SD card slot is located on the bottom on the side opposite the USB ports.

    <a class="image-link" href="https://projects-static.raspberrypi.org/projects/raspberry-pi-setting-up/94c43714c0e0536158409093ba28931e0fa5c9bc/en/images/pi-sd.png">
    <img src="https://projects-static.raspberrypi.org/projects/raspberry-pi-setting-up/94c43714c0e0536158409093ba28931e0fa5c9bc/en/images/pi-sd.png" alt="Image from RPi Foundation" style="max-width:300px;"/>
    </a>

2. Turn on the power bank and ensure that the power cables are connected as per the [assembly guide](https://mbot.robotics.umich.edu/docs/hardware/classic/assembly/mbot-wiring/).
3. **Wait for about 1 minute until the OLED screen lights up** like the image below, where the IP should be something like `192.168.X.X` meaning it is part of a local network and not directly accessible from the public internet. Once you see this, you can proceed to the next step.

    <a class="image-link" href="/assets/images/botlab/system-setup-oled.jpg">
    <img src="/assets/images/botlab/system-setup-oled.jpg" alt="" style="max-width:300px;"/>
    </a>


### 4.1 Connect to the Internet on Campus
1. Connect to the MBot's local access point. See the instructions on the official MBot website [here](https://mbot.robotics.umich.edu/docs/setup/networking/#connecting-to-the-mbots-access-point) under the "Connecting to the MBot’s Access Point" section.

2. Use NoMachine to connect to MBot. Follow the instructions on the official MBot website [here](https://mbot.robotics.umich.edu/docs/tutorials/no-machine/).
    - **At the second step**, you can:
        1. Either eneter the numeric IP address on the OLED screen.
        2. Or enter `<MBOT-HOSTNAME>.local` like in the video demo.
    - Note: NoMachine can be SLOW so please be PATIENT. In the video demo below, we reopen NoMachine a few times due to delays. If you feel it's taking too long, you can try reopening the connection as well. you will eventually get through.

3. Connect the MBot to the Internet. Open a terminal in the NoMachine desktop and run the following commands to connect the MBot to the Internet:
    ```bash
    cd ~
    ./SecureW2_JoinNow.run
    ```

4. When prompted for your unique name and password, use the 550 course credentials, it will be distributed during lab. Avoid using your own UM credentials for your privacy safety. You will be disconnected from the NoMachine session, but that is normal.

5. After entering your credentials, NoMachine will disconnect during the process. Close the NoMachine window and wait for about 1 minute. Check the OLED screen to see if the IP address has changed from `192.168.X.X` to a different one. This indicates that the MBot has successfully connected to the Internet. If the OLED screen says "IP Not Found" and has been that way for over two minutes, you aren't connected to the network, so come see a GSI and ask to connect to an external monitor.

**Video Demo:**
<iframe width="560" height="315" src="https://www.youtube.com/embed/Fw_pE00xbsA?si=eVlwtsD_uPoY55II" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

**If the above steps do not work for you, you will need to find an external monitor, keyboard, and mouse, and then start from step 3.**

### 4.2 Connect to the Internet at Home
1. Ensure the `new_wifi_ssid` and `new_wifi_password` are correctly set in the `mbot_config.txt` file (check for any typos). They should be your home wifi name, and home wifi password.

2. Start the MBot, wait for about 1~2 minute, please be patient. You may hear the fan start a couple of times, and eventually, the OLED screen will display the MBot's IP address (since you are at home, the IP be `192.168.x.x` is normal unlike on campus), meaning the mbot is connected to your home Wi-Fi.


If you have successfully connected your MBot to MWireless or your home Wi-Fi, you can use its IP address to remotely access the MBot. From now on, you can always find the IP address on the OLED screen. Proceed to the next step for more details.

### 5. Remote Access
**Upon this step, your laptop is now just a gateway for the SSH connection to your MBot. All programming is executed on the MBot, not on your laptop. When we mention opening a terminal in this guide later, we're referring to using a VSCode terminal to access your MBot.**
{: .text-red-200}

To access the mbot, your laptop and the MBot must always be on the same network. And there are 2 options to connect:
1. Using VSCode (Recommended), here is the tutorail: [link](https://mbot.robotics.umich.edu/docs/tutorials/vscode/#connecting-to-the-robot)
2. Using NoMachine, here is the tutorail: [link](https://mbot.robotics.umich.edu/docs/tutorials/no-machine/)

Username: mbot <br>
Password: i<3robots!
{: .note }


### 6. Change your mbot's password

For mbot network security, we encourage you to change your password once you have set up your mbot. The default password is `i<3robots!`, which everyone uses. You should set your password to the default password followed by your unique name. Here's how to change your mbot's password:
1. Open a VSCode terminal on your laptop.
2. Enter this command: `passwd`. You will be prompted to enter your current password.
3. Next, you will be asked to enter your new password and retype it to confirm.
4. If the passwords match, you will see a message indicating that your password has been updated successfully.

The output will look like this:
```bash
mbot@mbot-0018-shaw:~ $  passwd
Changing password for mbot.
Current password:
New password:
Retype new password:
passwd: password updated successfully
```

{: .highlight }
Now you have completed all the basic setup for MBot! Next, we are going to install all the codes.

## Set up MBot firmware
> In this session, we are going to work on setup of the Control Board.

### 1. Compile the firmware files
1. Firstly, connect to the MBot and open a terminal, then run:
    ```bash
    # move to the home directory
    cd ~
    # create a new folder called mbot_ws
    mkdir mbot_ws
    # navigate to mbot_ws
    cd ~/mbot_ws
    ```

2. Clone [mbot_lcm_base](https://gitlab.eecs.umich.edu/ROB550-F24/mbot_lcm_base) to `~/mbot_ws` and install.
    1. Clone from Github
        ```bash
        cd ~/mbot_ws/
        git clone https://gitlab.eecs.umich.edu/ROB550-F24/mbot_lcm_base
        ```
    2. Install lcm related stuff

        ```bash
        cd ~/mbot_ws/mbot_lcm_base
        ./scripts/install.sh
        ```

3. As in Armlab, one group member needs to create a GitLab Group and add all members. Follow these steps:
    
    1. On the left-hand menu of the GitLab webapp, click on "Groups" to access the [Groups page](https://gitlab.eecs.umich.edu/dashboard/groups), then click the "New Group" button to initiate the group creation process.
    2. Group Details:
        - Group name: enter `botlab-s<SECTION#>_g<GROUP #>` (i.e. botlab-s012_g7)
        - Visibility level: select “Private”
        - Leave other fields blank
    3. Invite your team members as “owners” of the project
        - Firstly, go to groups page: sidebar -> groups
        - Click your team group
        - Add members: sidebar -> Manage -> Members -> invite members

4. Next, **fork** [mbot_firmware](https://gitlab.eecs.umich.edu/ROB550-F24/mbot_firmware) to your group first, as you will need to modify them for course assignment later, **then clone** your forked codebase to the Pi5 in `~/mbot_ws`.

5. Compile the firmware code to get .uf2 binary files

    1. Run the firmware setup script
        ```bash
        cd ~/mbot_ws/mbot_firmware
        ./setup.sh
        ```
    2. Build firmware
        ```bash
        cd ~/mbot_ws/mbot_firmware
        mkdir build
        cd build
        cmake -DMBOT_TYPE=CLASSIC -DENC=48 ..
        make
        ```

### 2. Calibrate the MBot and flash the firmware
In this step, we are going to flash the calibration script onto the Pico to calibrate it and then flash the firmware.

1. **Place the MBot on the floor** in a spot with at least 2 feet of clear space around the robot, preferably on the same type of surface you plan to use it on.
2. Follow the [firmware flashing guide](https://mbot.robotics.umich.edu/docs/setup/firmware/). There are multiple ways to flash the firmware, "Manual Boot Mode" is recommended.
3. First, flash the calibration file we compiled from step 1, to calibrate your MBot. The calibration file is named something like `mbot_calibrate_classic_<vX.X.X>_enc48.uf2` under the `build` folder. Here is a video showing the expected calibration routine:

    <iframe width="400" height="227" src="https://www.youtube.com/embed/iIghZzf8ZQY?si=w_ultg6PtCwJVr2_" title="YouTube video player" frameborder="2" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
    - **Allow the Pico to finish its calibration routine without interference.** The calibration script will save the parameters onto the Pico’s memory.
4. Then, you can flash the firmware that will run on the Pico during operation. The firmware file name should be `mbot_classic_<vX.X.X>_enc48.uf2`.


### 3. Using Minicom to verify
Minicom is a program designed for serial communication that connects devices to a Linux PC via serial ports, we will use Minicom to read the pico printouts during the flash process.
- After flashing the firmware to the Pico, run the following command to start minicom
    ```bash
    minicom -D /dev/mbot_tty -b 115200
    ```
    - If the minicom command doesn't work, run this command in the Terminal: `ls /dev | grep mbot`
    ```bash
    mbot@mbot-0018-shaw:/dev$ ls /dev | grep mbot
    mbot_lcm
    mbot_tty
    ```
    If you do not see the 2 outputs above, unplug the USB which connect Pi5 and Pico, then plug back in.
    - **Successful Firmware Flashing:** After flashing the firmware successfully, Minicom will display a table, showing the encoder counts, IMU values, and more. Manually turning the wheel will update the encoder counts in the Minicom terminal.
    - **Unsuccessful Firmware Flashing:** If the firmware doesn't flash correctly, repeat the calibration and firmware flashing steps. This time, open a second terminal window with Minicom running to monitor its outputs for troubleshooting.

- **To exit Minicom**, press `CTRL-A`, then press `X`, then press `Enter` to quit.


**Note:**<br>
To make running minicom easier, consider creating a [permanent alias](https://askubuntu.com/questions/154640/how-to-add-an-alias-to-a-command-in-terminal) by editing your .bashrc file. Run the following commands in your terminal:
```bash
echo "alias start-minicom='minicom -D /dev/mbot_tty -b 115200'" >> ~/.bashrc
source ~/.bashrc
```
This will let you run minicom using the command `start-minicom`.


## Install the rest of the MBot Code

1. Clone [RP Lidar Driver](https://github.com/mbot-project/rplidar_lcm_driver) and [MBot Bridge](https://github.com/mbot-project/mbot_bridge.git) to your Pi5 under folder `mbot_ws`
    ```bash
    cd ~/mbot_ws/
    git clone https://github.com/mbot-project/rplidar_lcm_driver.git
    git clone https://github.com/mbot-project/mbot_bridge.git
    ```
2. Install the MBot Bridge and the RPLidar driver
    ```bash
    cd ~/mbot_ws/mbot_bridge/
    ./scripts/install.sh

    cd ~/mbot_ws/rplidar_lcm_driver/
    ./scripts/install.sh
    ```
3. Install the MBot Web App
    1. Download the latest web app release and unpack it
    ```bash
    cd ~/mbot_ws
    wget https://github.com/mbot-project/mbot_web_app/releases/download/v2.0.0/mbot_web_app-v2.0.0.tar.gz
    tar -xvzf mbot_web_app-v2.0.0.tar.gz
    ```
    2. Install the web app dependencies
    ```bash
    cd mbot_web_app-v2.0.0/
    ./install_nginx.sh
    ./install_python_deps.sh
    ```

    3. Build and install the app
    ```bash
    ./deploy_app.sh --no-rebuild
    ```

    {: .highlight }
    You can now use the web app by going to your browser and typing in the robot’s IP address.
    - If the firmware is flashed and the serial server is running, you should be able to drive the robot through the web app. Toggle "Drive Mode" on, then use the W, S, Q, and E keys to control the robot.
    - You don't have to re-run any of these steps, the web app will automatically start when you start the robot.
4. Restart the MBot

    Turn the power bank off and then back on. Open the web app and check if you can still drive the MBot.
    - **If the robot responds correctly,**
        - Your MBot setup is complete, and you're ready for development.
        - To save space, you can now safely delete all the web app related code by running:
            ```bash
            cd ~/mbot_ws
            rm mbot_web_app-v1.3.0.tar.gz
            rm -r mbot_web_app-v1.3.0/
            ```
    - **If the robot does not respond,**
        - Start minicom and check if you can see the table output in the terminal. If minicom shows no output, re-flash the firmware (not the calibration script) while keeping minicom running in a second terminal. Once the flashing is complete, restart the MBot and repeat this step. If the robot still does not respond, ask the instructors for help.



5. **Fork** [MBot Autonomy](https://gitlab.eecs.umich.edu/ROB550-F24/mbot_autonomy) to your group and **then clone** the forked code to Pi5, then install the MBot Autonomy code
    ```bash
    cd ~/mbot_ws/mbot_autonomy
    mkdir build
    cd build
    cmake -DMBOT_TYPE=DIFF ..
    make
    ```

6. Clone [mbot_gui](https://github.com/mbot-project/mbot_gui) and install it.
    ```bash
    cd ~/mbot_ws
    git clone https://github.com/mbot-project/mbot_gui.git
    sudo apt install libgtk2.0-dev \
                     mesa-common-dev \
                     libgl1-mesa-dev \
                     libglu1-mesa-dev \
                     libusb-dev libusb-1.0-0-dev \
                     libdc1394-dev libgsl-dev
    
    cd mbot_gui
    mkdir build
    cd build
    cmake .. && make
    ```
