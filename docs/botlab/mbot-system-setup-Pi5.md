---
layout: default
title: MBot System Setup
parent: Botlab
nav_order: 2
last_modified_at: 2024-08-15 11:59:00 -0500
---

{: .important}
This guide is for MBot Classic using **Raspberry Pi 5**!

> This guide will walk you through the steps needed to setup the MBot Classic system. The guide is intended to be followed in order, do not jump back and forth.

The following items are needed:
1. microSD card for main storage
2. SD adapter
3. USB keyboard and mouse (Optional)


### Contents
- TOC
{:toc}

## Set up RPi 5 System
### 1. Flash the image
1. Download the custom Pi5 image `2024-10-16-mbot-base-bookworm.img` from this [link](https://www.dropbox.com/scl/fi/tyyslhj3fz7dd2y6267mp/2024-10-16-mbot-base-bookworm.img.gz?rlkey=4k3qe7knhqhabw4g17n9feukj&st=fdf77uva&dl=0). We use a custom image with RPiOS based on Debian 12 Bookworm
2. Download [Balena Etcher](https://etcher.balena.io/) then flash the OS image to your SD card. Plug in the SD card to your laptop using SD card reader then following the steps in Balena Etcher

You now have an SD card with the OS image flashed on it for the Pi5. Keep the card in your laptop for now and proceed to the next step.

{: .warning }
If you do the flashing on a Windows computer, you may see many file explorer windows and error messages pop up when you insert the SD card and when you finish flashing. Those are expected, and you can safely close the file explorer windows and dismiss the error messages. However, if Windows asks you to format the SD card through a popup dialog box, close the message through the "Cancel" button and **do not** click the "Format Disk" button.


### 2. Set up system utilities

If the flash succeeded, the SD card will have two partitions: a 134MB Volume formatted as fat32 and a 16GB Volume formated as ext4. When you insert the SD card in your laptop, it should mount the smaller fat32 partition.  Find the file `mbot_config.txt` on this volume and modify it as follows:
- Set `mbot_hostname` following this format: `mbot-<section>-<team#>-<unique_name>`
    - For example: if you are in the AM section team 6, and your unique_name is johndoe, you should name the robot as `mbot-AM-team6-johndoe`
- Enter your home Wi-Fi details for `new_wifi_ssid` and `new_wifi_password` if you intend to use it at home later.
- Leave all other variables unchanged.

### 3. Boot the Pi5
1. Insert the SD card into your Pi5. The SD card slot is located on the bottom on the side opposite the USB ports.

    <a class="image-link" href="https://projects-static.raspberrypi.org/projects/raspberry-pi-setting-up/94c43714c0e0536158409093ba28931e0fa5c9bc/en/images/pi-sd.png">
    <img src="https://projects-static.raspberrypi.org/projects/raspberry-pi-setting-up/94c43714c0e0536158409093ba28931e0fa5c9bc/en/images/pi-sd.png" alt="Image from RPi Foundation" style="max-width:300px;"/>
    </a>

2. Turn on the power bank and ensure that the power cables are connected as per the assembly guide.

### 4. Connect to the Internet
If you are on campus using MWireless:
1. Connect in NoMachine to the local accesspoint.
2. Open a terminal and run:
    ```bash
    $ cd ~
    $ SecureJoinNow.run
    ```
3. Connect with the credentials for the course given by Instructor.
4. NoMachine will disconnect in the process. Wait 1 minute then reboot the robot.

If you are at home:


### 5. Remote Access
Now the mbot is online, you can remote access to it.

You have 2 options:
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
Now you have completed all the setup for Pi5!

**Note: <br> With the setup now complete, your laptop is now just a gateway for the SSH connection to your MBot. All programming is executed on the MBot, not on your laptop. When we mention opening a terminal in this guide later, we're referring to using a VSCode terminal to access your MBot.**
{: .text-red-200}

## Set up MBot firmware
> In this session, we are going to work on setup of the Control Board.

### 1. Compile the firmware files
1. Firstly navigate to `mbot_ws`:
    ```bash
    $ cd ~
    $ mkdir mbot_ws
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
        $ cmake ..
        $ make
        ```
    3. Now you will have 2 relevant `.uf2` files under `/build`
        - The calibration script, `mbot_firmware/build/mbot_calibrate_classic_v1.0.0_enc48.uf2`
        - The MBot firmware, `mbot_firmware/build/mbot_classic_v1.0.0_enc48.uf2`

        The name is long, you can hit `Tab` key on the keyboard to auto complete.


### 2. Calibrate the MBot and flash the firmware
In this step, we are going to flash the calibration script onto the Pico to calibrate it before we flash the firmware.

1. Place the MBot on the floor in a spot with at least 2 feet of clear space all around the robot, preferably on the same type of surface that you plan to use the robots on.
2. Run the following command, the Pico will reboot automatically, and will then run its calibration routine right away. Allow the Pico to finish its calibration routine without interference.

    {: .warning}
    Hold off on running this command until the robot is placed on the floor.

    ```bash
    $ cd ~/mbot_ws/mbot_firmware
    # upload the calibration scripts
    $ sudo ./upload.sh flash build/build/mbot_calibrate_classic_v1.0.0_enc48.uf2
    ```

    Here is a video of expected routine:

    <iframe width="400" height="227" src="https://www.youtube.com/embed/iIghZzf8ZQY?si=w_ultg6PtCwJVr2_" title="YouTube video player" frameborder="2" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

The calibration script will have saved parameters onto the Pico’s memory. We can now flash the firmware that will run on the Pico during operation.

```bash
$ cd ~/mbot_ws/mbot_firmware
$ sudo ./upload.sh flash build/mbot_classic_v1.0.0_enc48.uf2
```

### 3. Using Minicom to verify
Here we introduce you the tool Minicom. It is a program designed for serial communication that connects devices to a Linux PC via serial ports, we will use Minicom to read the pico printouts from the Pi5.

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
    If you do not see the 2 outputs above, unplug the USB which connect Pi5 and Pico, then plug back in.
- To exit Minicom, press `CTRL-A` to get to command mode, then press `X` to quit.

**Successful Firmware Flashing:** After flashing the firmware successfully, Minicom will display your encoder counts, IMU values, and more.
Manually turning the wheel will update the encoder counts in the Minicom terminal.

**Unsuccessful Firmware Flashing:** If the firmware doesn't flash correctly, repeat the calibration and firmware flashing steps.
Open a second terminal window with Minicom to monitor its outputs for troubleshooting.

{: .note }
To make running minicom easier, consider creating a [permanent alias](https://askubuntu.com/questions/154640/how-to-add-an-alias-to-a-command-in-terminal) by editing your .bashrc file.
Add the line `alias start-minicom='minicom -D /dev/mbot_tty -b 115200'` to the end of your .bashrc file, then run the command `source ~/.bashrc` in a terminal. This will let you run minicom using the command `start-minicom`.

### 4. Manually enter bootloader mode

If the firmware was successfully flashed, skip this step and proceed to the next.

If your firmware flashing was not successful because the `./upload.sh` script from step two doesn't work for your mbot, you can manually enter the bootloader mode (namely let pico ready to get the firmware) by following instructions [here](/docs/botlab/how-to-guide/manually-enter-bootloader).


## Install the rest of the MBot Code

1. Clone the necessary repos to your Pi5 under folder `mbot_ws`
    - **Clone** [RP Lidar Driver](https://github.com/mbot-project/rplidar_lcm_driver) and [MBot Bridge](https://github.com/mbot-project/mbot_bridge.git)
    - **Fork** [MBot Autonomy](https://gitlab.eecs.umich.edu/rob550-f24/mbot_autonomy) to your group and **then clone** the forked code to Pi5

2. Install the MBot Bridge
    ```bash
    $ cd ~/mbot_ws/mbot_bridge/
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
    The web app should now be available! You don't have to re-run any of these steps; the web app will automatically start when you start the robot.

    {: .note }
    You can use the web app by going to your browser and typing in the robot’s IP address. <br>
    If the firmware is flashed and the serial server is running, you should be able to drive the robot through the webapp. Toggle drive mode on then use the keys WSQE to drive the robot.

4. Install the RPLidar driver
```bash
$ cd ~/mbot_ws/rplidar_lcm_driver/
$ ./scripts/install.sh
```
5. Install the MBot Autonomy code
```bash
$ cd ~/mbot_ws/mbot_autonomy/
$ ./scripts/install.sh
```
