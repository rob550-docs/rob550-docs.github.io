---
layout: default
title: MBot System Setup
parent: Botlab
nav_order: 2
last_modified_at: 2025-10-20 14:38:00 -0500
---

{: .new}
This guide has been updated on Oct. 20!

{: .important}
This guide has been updated for **ROS2** MBot Classic!


The following items are needed:
1. microSD card
2. SD adapter
3. A laptop can read and write SD card

**Notice #1**: You do not need an external monitor during the set up process. However, if you want to connect to external monitor at any point, you need to plug into HDMI-0 port. The Pi5 has 2 HDMI ports, plug into the one closer to USB C port.

**Notice #2**: This guide will walk you through the steps needed to setup the MBot Classic system. The guide is intended to be followed in order, do not jump back and forth.

### Contents
- TOC
{:toc}

## Set up RPi 5 System
### 1. Flash the image
1. Download the custom image `2025-08-15-mbot-ros2-base-ubuntu24.img.gz` from this [link](https://www.dropbox.com/scl/fi/urrfxt71np1r4875pukzo/2025-08-15-mbot-ros2-base-ubuntu24.img.gz?rlkey=zztcp8wjjofmft918imdroptg&st=gvhvc8qx&dl=0) to your laptop.
2. Download [Balena Etcher](https://etcher.balena.io/) to your laptop, it is a tool to flash the OS image to the SD card. Plug in the SD card to your laptop using SD card reader then following the steps in Balena Etcher. (The image is 18GB, this can take 15 min).

Once you have the SD card with the OS image flashed on it, don't insert the SD card in the Pi 5 yet, continue to the next step.

{: .warning }
If you do the flashing on a Windows computer, you may see many file explorer windows and error messages pop up when you insert the SD card and when you finish flashing. Those are expected, and you can safely close the file explorer windows and dismiss the error messages. However, if Windows asks you to format the SD card through a popup dialog box, close the message through the "Cancel" button and **do not** click the "Format Disk" button.


### 2. Set up system utilities

If the flash was successful, insert the SD card into your laptop, you should see a folder named “system-boot” or something similar.

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
**Video Demo:**
<iframe width="560" height="315" src="https://www.youtube.com/embed/FF700eSdS6g?si=5K4xI6k8Y4ttyCH1" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

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

5. After entering your credentials, NoMachine will disconnect during the process. Close the NoMachine window and wait for about 1 minute. Check the OLED screen to see if the IP address has changed from `192.168.X.X` to a different one. This indicates that the MBot has successfully connected to the Internet. 
- If the OLED screen says "IP Not Found" or "Error" and has been that way for over two minutes, you aren't connected to the network, you might have typo when input the UM account password, find GSIs and ask to connect to an external monitor.

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

## Flash MBot firmware
> In this session, we are going to work on setup of the Control Board.

### 1. Download the Firmware Files
1. Go to the GitLab [botlab_uf2](https://gitlab.eecs.umich.edu/rob550-f25/botlab_uf2) repository.
2. Clone the repository to home directory.
    ```bash
    cd ~
    git clone https://gitlab.eecs.umich.edu/rob550-f25/botlab_uf2.git
    ```

### 2. Calibrate the MBot and flash the firmware
In this step, we are going to flash the calibration script onto the Pico to calibrate it and then flash the firmware.

1. **Place the MBot on the floor** in a spot with at least 2 feet of clear space around the robot, preferably on the same type of surface you plan to use it on.
2. First, flash the calibration file to calibrate your MBot.
    ```bash
    cd ~/botlab_uf2
    sudo mbot-upload-firmware flash mbot_calibrate_classic_v1.1.1.uf2
    ```
    Expected output:
    ```bash
    $ sudo mbot-upload-firmware flash mbot_calibrate_classic_v1.1.1.uf2
    [sudo] password for mbot: 
    Detected Raspberry Pi 5, entering flash mode...
    Flashing action for mbot_calibrate_classic_v1.1.1.uf2...
    Loading into Flash:   [==============================]  100%
    The device was rebooted into application mode.
    ```

    Here is a video showing the expected calibration routine:

    <iframe width="400" height="227" src="https://www.youtube.com/embed/iIghZzf8ZQY?si=w_ultg6PtCwJVr2_" title="YouTube video player" frameborder="2" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
    - **Allow the Pico to finish its calibration routine without interference.** The calibration script will save the parameters onto the Pico’s memory.
3. Then, you can flash the firmware that will run on the Pico during operation.
    ```bash
    sudo mbot-upload-firmware flash mbot_classic_ros_v1.1.1.uf2 
    ```


### 3. Using Minicom to verify
Minicom is a program designed for serial communication that connects devices to a Linux PC via serial ports, we will use Minicom to read the pico printouts during the flash process.
- After flashing the firmware to the Pico, run the following command to start minicom
    ```bash
    sudo minicom -D /dev/ttyACM0 -b 115200
    ```
    You should see output like this:
    ```bash
    |-----------------------------------------------|
    | ANALOG                                        |
    |  AIN 0    |  AIN 1    |  AIN 2    |  BATT (V) |
    |-----------|-----------|-----------|-----------|
    |    0.0088 |    0.0095 |    0.3743 |   10.4993 |
    |-----------------------|
    | ENCODERS              |
    |  ENC L    |  ENC R    |
    |-----------|-----------|
    |         1 |        -1 |
    |-----------------------------------|
    | IMU                               |
    |  ROLL     |  PITCH    |  YAW      |
    |-----------|-----------|-----------|
    |   -0.1026 |    0.0616 |    0.0044 |
    |-----------------------|
    | MOTOR                 |
    |  MOT L    |  MOT R    |
    |-----------|-----------|
    |    0.0000 |    0.0000 |
    |-----------------------------------|
    | ODOMETRY                          |
    |  X        |  Y        |  THETA    |
    |-----------|-----------|-----------|
    |    0.0000 |    0.0000 |    0.0000 |
    |-----------------------------------|
    ```
    - **Successful Firmware Flashing:** After flashing the firmware successfully, Minicom will display a table, showing the encoder counts, IMU values, and more. Manually turning the wheel will update the encoder counts in the Minicom terminal.
    - **Unsuccessful Firmware Flashing:** If the firmware doesn't flash correctly, repeat the calibration and firmware flashing steps. This time, open a second terminal window with Minicom running to monitor its outputs for troubleshooting.

- **To exit Minicom**, press `CTRL-A`, then press `X`, then press `Enter` to quit.

## Drive the mbot
Run the following command in the VSCode terminal:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
- Press `x` a few times to reduce the linear speed to 0.2, so the robot doesn't tip over.
- Use your keyboard to drive the robot. As shown in the terminal:
    - `i` = forward
    - `,` = backward
    - `j` = turn left
    - `l` = turn right

If you’ve successfully driven your robot around, your control board setup is complete!

## Setup mbot_ws
1. Fork the [mbot_ws code](https://gitlab.eecs.umich.edu/rob550-f25/mbot_ros2_ws) to your group, and clone it to your mbot home directory.
   ```bash
    cd ~
    mkdir mbot_ws
    cd ~/mbot_ws
    git clone --recurse-submodules your_group_url src
   ```
    - The repository is called `mbot_ros2_ws`, but here we clone it into `~/mbot_ws/src`, the repo's name is irrelevant.
2. Run the following commands:
    ```bash
    cd ~/mbot_ws
    # install ros dependencies
    rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera
    # build the packages
    colcon build --symlink-install
    # source the env
    echo "source $PWD/install/local_setup.bash" >> ~/.bashrc
    source install/local_setup.bash
    ```
    - There might be warnings after running colcon build from sllidar_ros2 package, it is normal, if you re-run the command the warnings will be gone.

### Test Camera

Test your camera by going through the following steps in order. If any of the steps produce unexpected results, contact the GSIs for assistance.

1. To test if camera is detected, run:
   ```bash
   rpicam-hello
   ```
   - If the terminal didn't say "ERROR: *** no cameras available ***", your camera is detected. Everything is good, use `ctrl+C` to quit.
2. To test if camera is working, run:
   ```bash
   cd ~
   rpicam-still -t 1 -o test.jpg
   ```
   - This will take a photo and save it in the home directory, you can use vscode to check the photo. The photo will be upside-down, that's expected.
3. To test if camera ROS driver is working, run in vscode terminal:
   ```bash
    # bring up the camera node and flip the image
    ros2 run camera_ros camera_node --ros-args \
    -p orientation:=180 \
    -p width:=640 -p height:=480 \
    -p format:=BGR888
   ```
   - Ignore the errors about calibration files.
   - You can visualize the camera view in rqt on NoMachine desktop. Run the following in NoMachine Terminal:
    ```bash
    ros2 run rqt_image_view rqt_image_view
    ```
    - Select the image topic `/camera/image_raw`

### Test LiDAR
1. Run the following in vscode terminal:
    ```bash
    ros2 launch mbot_bringup mbot_bringup.launch.py 
    ```
    - This will bring up the lidar driver, the robot description for visualization, and the static tf.
2. Run the following on NoMachine:
   ```bash
   ros2 launch mbot_bringup mbot_viz.launch.py
   ```
   You should see the rviz with the robot model and the LiDAR scan show up.

    <a class="image-link" href="/assets/images/botlab/rviz0.png">
    <img src="/assets/images/botlab/rviz0.png" alt="Image from RPi Foundation" style="max-width:600px;"/>
    </a>


If you have successfully completed both tests, your MBot setup is complete, which means Checkpoint 0 is finished. Make sure everything is working properly before the lab starts Checkpoint 1.