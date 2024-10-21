---
layout: default
title: MBot System Setup
parent: Botlab
nav_order: 2
last_modified_at: 2024-10-17 12:59:00 -0500
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

## Set up RPi 5 System
### 1. Flash the image
1. Download the custom Pi5 image `2024-10-16-mbot-base-bookworm.img` from this [link](https://www.dropbox.com/scl/fi/tyyslhj3fz7dd2y6267mp/2024-10-16-mbot-base-bookworm.img.gz?rlkey=4k3qe7knhqhabw4g17n9feukj&st=1knc90ky&dl=0) to your laptop. We use a custom image with RPiOS based on Debian 12 Bookworm
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

    <a class="image-link" href="https://mbot.robotics.umich.edu/assets/images/tutorials/mbot-oled-ip.jpg">
    <img src="https://mbot.robotics.umich.edu/assets/images/tutorials/mbot-oled-ip.jpg" alt="Image from RPi Foundation" style="max-width:300px;"/>
    </a>


### 4.1 Connect to the Internet on Campus
1. Connect to the MBot's local access point. See the instructions on the official MBot website [here](https://mbot.robotics.umich.edu/docs/setup/networking/#connecting-to-the-mbots-access-point) under the "Connecting to the MBot’s Access Point" section.

2. Use NoMachine to connect to MBot. Follow the instructions on the official MBot website [here](https://mbot.robotics.umich.edu/docs/tutorials/no-machine/).
    - **At the second step**, you can:
        1. Either eneter the numeric IP address on the OLED screen.
        2. Or enter `<MBOT-HOSTNAME>.local` like in the video demo.
    - Note: NoMachine can be SLOW so please be EXTRA PATIENT. In the video demo below, we reopen NoMachine a few times due to delays. If you feel it's taking too long, you can try reopening the connection as well. you will eventually get through.

3. Connect the MBot to the Internet. Open a terminal in the NoMachine desktop and run the following commands to connect the MBot to the Internet:
    ```bash
    $ cd ~
    $ ./SecureW2_JoinNow.run
    ```

4. When prompted for your unique name and password, use the course credentials provided by your instructor (for Fall 2024, these will be sent via Discord).

5. After entering your credentials, NoMachine will disconnect during the process. Close the NoMachine window and wait for about 1 minute. Check the OLED screen to see if the IP address has changed from `192.168.X.X` to a different one. This indicates that the MBot has successfully connected to the Internet.

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
$  passwd
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
    $ cd ~
    # create a new folder called mbot_ws
    $ mkdir mbot_ws
    # navigate to mbot_ws
    $ cd ~/mbot_ws
    ```

2. Clone [mbot_lcm_base](https://github.com/mbot-project/mbot_lcm_base.git) to `~/mbot_ws` and install.
    1. Clone from Github
        ```bash
        $ cd ~/mbot_ws/
        $ git clone https://github.com/mbot-project/mbot_lcm_base.git
        ```
    2. Install lcm related stuff

        ```bash
        $ cd ~/mbot_ws/mbot_lcm_base
        $ ./scripts/install.sh
        ```

3. Next, **fork** [mbot_firmware](https://gitlab.eecs.umich.edu/ROB550-F24/mbot_firmware) to your group first, you will need to modify them for course assignment later, **then clone** your forked codebase to the Pi5 in `~/mbot_ws`.

4. Compile the firmware code to get .uf2 binary files

    1. Run the firmware setup script
        ```bash
        $ cd ~/mbot_ws/mbot_firmware
        $ ./setup.sh
        ```
    2. Build firmware
        ```bash
        $ cd ~/mbot_ws/mbot_firmware
        $ mkdir build
        $ cd build
        $ cmake -DMBOT_TYPE=CLASSIC -DENC=48 ..
        $ make
        ```

### 2. Calibrate the MBot and flash the firmware
In this step, we are going to flash the calibration script onto the Pico to calibrate it before we flash the firmware.

1. **Place the MBot on the floor** in a spot with at least 2 feet of clear space all around the robot, preferably on the same type of surface that you plan to use the robots on.
2. Run the following command, the Pico will reboot automatically, and will then run its calibration routine right away. **Allow the Pico to finish its calibration routine without interference.**

    {: .warning}
    Hold off on running this command until the robot is placed on the floor.

    ```bash
    $ cd ~/mbot_ws/mbot_firmware
    # upload the calibration scripts
    $ sudo mbot-upload-firmware flash build/mbot_calibrate_classic_v1.1.0_enc48.uf2
    ```
    - The name is long, you can hit `Tab` key on the keyboard to auto complete.
    - **If the MBot does not move after running the command above**, wait a few seconds. If it still does not move, you can **manually enter the bootloader mode** (namely let pico ready to get the firmware) by following instructions [here](/docs/botlab/how-to-guide/manually-enter-bootloader).

    Here is a video of expected routine:

    <iframe width="400" height="227" src="https://www.youtube.com/embed/iIghZzf8ZQY?si=w_ultg6PtCwJVr2_" title="YouTube video player" frameborder="2" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

The calibration script will have saved parameters onto the Pico’s memory. We can now flash the firmware that will run on the Pico during operation.

```bash
$ cd ~/mbot_ws/mbot_firmware
$ sudo mbot-upload-firmware flash build/mbot_classic_v1.1.0_enc48.uf2
```

### 3. Using Minicom to verify
Minicom is a program designed for serial communication that connects devices to a Linux PC via serial ports, we will use Minicom to read the pico printouts during the flash process.
- After flashing the firmware to the Pico, run the following command to start minicom
    ```bash
    $ minicom -D /dev/mbot_tty -b 115200
    ```
    - If the minicom command doesn't work, run this command in the Terminal: `ls /dev | grep mbot`
    ```bash
    $ mbot@mbot-0018-shaw:/dev$ ls /dev | grep mbot
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
$ echo "alias start-minicom='minicom -D /dev/mbot_tty -b 115200'" >> ~/.bashrc
$ source ~/.bashrc
```
This will let you run minicom using the command `start-minicom`.


## Install the rest of the MBot Code

1. Clone [RP Lidar Driver](https://github.com/mbot-project/rplidar_lcm_driver) and [MBot Bridge](https://github.com/mbot-project/mbot_bridge.git) to your Pi5 under folder `mbot_ws`
    ```bash
    $ cd ~/mbot_ws/
    $ git clone https://github.com/mbot-project/rplidar_lcm_driver.git
    $ git clone https://github.com/mbot-project/mbot_bridge.git
    ```
2. Install the MBot Bridge and the RPLidar driver
    ```bash
    $ cd ~/mbot_ws/mbot_bridge/
    $ ./scripts/install.sh

    $ cd ~/mbot_ws/rplidar_lcm_driver/
    $ ./scripts/install.sh
    ```
3. Install the MBot Web App
    1. Download the latest web app release and unpack it
    ```bash
    $ cd ~/mbot_ws
    $ wget https://github.com/mbot-project/mbot_web_app/releases/download/v1.3.0/mbot_web_app-v1.3.0.tar.gz
    $ tar -xvzf mbot_web_app-v1.3.0.tar.gz
    ```
    2. Install the web app dependencies
    ```bash
    $ cd mbot_web_app-v1.3.0/
    $ ./install_nginx.sh
    $ ./install_python_deps.sh
    ```

    3. Build and install the app
    ```bash
    $ ./deploy_app.sh --no-rebuild
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
            $ cd ~/mbot_ws
            $ rm mbot_web_app-v1.3.0.tar.gz
            $ rm -r mbot_web_app-v1.3.0/
            ```
    - **If the robot does not respond,**
        - Start minicom and check if you can see the table output in the terminal. If minicom shows no output, re-flash the firmware (not the calibration script) while keeping minicom running in a second terminal. Once the flashing is complete, restart the MBot and repeat this step. If the robot still does not respond, ask the instructors for help.



5. **Fork** [MBot Autonomy](https://gitlab.eecs.umich.edu/ROB550-F24/mbot_autonomy) to your group and **then clone** the forked code to Pi5, then install the MBot Autonomy code
```bash
$ cd ~/mbot_ws/mbot_autonomy/
$ ./scripts/install.sh -t DIFF --no-enable
```
