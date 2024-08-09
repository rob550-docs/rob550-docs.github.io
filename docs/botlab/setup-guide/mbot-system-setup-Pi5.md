---
layout: default
title: MBot System Setup with RPi5
parent: Setup Guide
grand_parent: Botlab
nav_order: 2
last_modified_at: 2024-08-9 11:59:00 -0500
---

> This guide will walk you through the steps needed to setup the MBot Classic system. The guide is intended to be followed in order, do not jump back and forth.

The following items are needed:
1. microSD card for main storage
2. SD adapter
Optional:
3. USB keyboard and mouse


### Contents
- TOC
{:toc}

## Set up RPi 5 System
### 1. Flash the image
1. Download the custom Pi5 image `mbot-RPi5-base-Aug24.img.xz` from this [link](https://www.dropbox.com/scl/fi/f9jijqd5jctr3jmo62873/mbot-RPi5-base-Aug24.img.xz?rlkey=eqbjgdkby7md3tfei1vl742ep&st=kqrqg87g&dl=0).
    - We use a custom image with RPiOS based on Debian 12 Bookworm
2. Download [Balena Etcher](https://etcher.balena.io/) then flash the OS image to your SD card. Plug in the SD card to your laptop using SD card reader then following the steps in Balena Etcher

You now have an SD card with the OS image flashed on it for the Pi5. Keep the card in your laptop for now and proceed to the next step.

{: .warning }
If you do the flashing on a Windows computer, you may see many file explorer windows and error messages pop up when you insert the SD card and when you finish flashing. Those are expected, and you can safely close the file explorer windows and dismiss the error messages. However, if Windows asks you to format the SD card through a popup dialog box, close the message through the "Cancel" button and **do not** click the "Format Disk" button.


### 2. Set up system utilities

If the flash succeeded, the SD card will have two partitions: a 134MB Volume formatted as fat32 and a 16GB Volume formated as ext4. When you insert the SD card in your laptop, it should mount the smaller fat32 partition.  Find the file `mbot_config.txt` on this volume and modify it as follows:
- Set `mbot_hostname` following this format: `mbot-<section>-<team#>-<unique_name>`
    - For example: if you are in the AM section team 6, and your unique_name is johndoe, you should name the robot as `mbot-AM-team6-johndoe`
- Enter your home Wi-Fi details for `new_wifi_ssid` and `new_wifi_password` if you intend to use it at home later.

### 3. Boot the Pi5
1. Insert the SD card into your Pi5. The SD card slot is located on the bottom on the side opposite the USB ports. 

    <a class="image-link" href="https://projects-static.raspberrypi.org/projects/raspberry-pi-setting-up/94c43714c0e0536158409093ba28931e0fa5c9bc/en/images/pi-sd.png">
    <img src="https://projects-static.raspberrypi.org/projects/raspberry-pi-setting-up/94c43714c0e0536158409093ba28931e0fa5c9bc/en/images/pi-sd.png" alt="Image from RPi Foundation" style="max-width:300px;"/>
    </a>

2. Turn on the power bank and ensure that the power cables are connected as per the assembly guide.
3. Please allow a minute or so for it to initialize. Once the initialization is complete, the OLED screen on the side of the mbot will display the MBot's IP address. This information is crucial for connecting to your MBot remotely.

### 4. VSCode Remote - SSH extension
> In this step, we are going to establish remote access using the VSCode extension. After this setup, you will be able to access the Pi5 remotely using your laptop.

1. Open VSCode on your laptop, and install the `Remote - SSH` extension

    <a class="image-link" href="/assets/images/botlab/system-setup/vscode_ssh1.png">
    <img src="/assets/images/botlab/system-setup/vscode_ssh1.png" alt=" " style="max-width:600px;"/>
    </a>

2. After installing the `Remote - SSH` extension, a new "Remote Explorer" icon will appear on the side panel. This is where you can add the SSH connection to your mbot. Click the New Remote `+` icon as shown below:
    >Note that your laptop needs to connect to MWireless as well if you are on campus.
    
    <a class="image-link" href="/assets/images/botlab/system-setup/vscode_ssh2.png">
    <img src="/assets/images/botlab/system-setup/vscode_ssh2.png" alt=" " style="max-width:600px;"/>
    </a>

3. When the prompt window pops up, enter the connection command: `ssh mbot@your_mbot_ip_address` as shown below. Your IP address can be found in the IP registry and on the OLED screen.

    <a class="image-link" href="/assets/images/botlab/system-setup/vscode_ssh3.png">
    <img src="/assets/images/botlab/system-setup/vscode_ssh3.png" alt=" " style="max-width:500px;"/>
    </a>

4. Select the default config file. Note that different operating systems may have different paths, but this isn't necessarily a problem. Here in the image, we select the one contains user name.

    <a class="image-link" href="/assets/images/botlab/system-setup/vscode_ssh4.png">
    <img src="/assets/images/botlab/system-setup/vscode_ssh4.png" alt=" " style="max-width:500px;"/>
    </a>

5. Navigate to the "Remote Explorer" tab and click the refresh button. You should see your Pi5's IP address listed under the SSH section, indicating that your connection has been saved. 
    - **To start a remote connection:** Click on "Connect in New Window" and enter the password `i<3robots!`. After this, your SSH session should be up and running.
    - **To end a remote connection:** Click on the tab at the bottom left corner of the VS Code window labeled SSH: xx.x.xxx.xx. A pop-up menu will appear with the option to "close remote connection".

{: .note }
Username: mbot <br>
Password: i<3robots!

{: .highlight }
At this point, you should be able to connect to your MBot using VSCode extension. 

- To use the mbot at home
    - Firstly turn on the mbot at home. It will automatically connect to your Wi-Fi if you entered your home Wi-Fi details in the mbot_config.txt file. Replicate the 5 steps above to add your mbot's IP address at home to your VSCode remote connections. Once done, you will be able to remotely access your mbot at home.


### 5. Remote Desktop access - NoMachine
> In this step, we are going to set up NoMachine access. Upon completion, you will be able to access the Pi5 with a Desktop UI. 

1. Download NoMachine to your laptop from the [official site](https://www.nomachine.com/).
    - NoMachine is a remote access software and it is pre-installed on our customized Pi5 image. 
 
2. Connect to Pi5 using NoMachine
    - Open NoMachine on your laptop, connect to Pi5 as shown in the image below. You will need your IP address for this step.

        <a class="image-link" href="/assets/images/botlab/system-setup/nomachine1.png">
        <img src="/assets/images/botlab/system-setup/nomachine1.png" alt=" " style="max-width:400px;"/>
        </a>
    - Finally, enter the username: `mbot`, password: `i<3robots!` to log in.
    - Note: if the NoMachine desktop freezes, you can always restart the robot by turning the power off and then back on.

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

## Update system utilities
With the setup now complete, you have the capability to connect to your MBot remotely through **the VSCode terminal**.

Firstly establish a remote connection to your MBot. Once the remote connection is set, open a terminal within VSCode. In the VSCode terminal, navigate to the home directory by entering the command `cd ~`. 

In the home directory, you will find a folder named `mbot_sys_utils`. The `mbot_sys_utils` has been updated after the OS image was generated, and we need to update the settings accordingly. This step is essential for the firmware set up later.

**Note: Your laptop is now just a gateway for the SSH connection to your MBot. All programming is executed on the MBot, not on your laptop. When we mention opening a terminal in this guide later, we're referring to using a VSCode terminal to access your MBot.**


1. Pull the latest changes from the `mbot_sys_utils` repository using the following commands: <br>
```bash
$ cd ~/mbot_sys_utils
$ git pull
```
2. Install the services manually:
```bash
$ cd ~/mbot_sys_utils/services
$ ./install_mbot_services.sh
```
3. After installation, power off the device, your VS Code connection will drop at this point, then turn the power back on and reconnect to the MBot using VS Code.

## Set up MBot firmware
> In this session, we are going to work on setup of the Control Board.

### 1. Compile the firmware files
1. It is always a good practice to create a dedicated workspace when working on large-scale projects. Open a new Terminal in the VSCode remote session, then create a new folder called `mbot_ws`:
    ```bash
    $ mkdir mbot_ws 
    ```

2.    Clone [mbot_lcm_base](https://github.com/mbot-project/mbot_lcm_base.git) to `~/mbot_ws` and install.
    
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

3. Next, **fork** [mbot_firmware](https://gitlab.eecs.umich.edu/rob550-f24/mbot_firmware) to your group first, you will need to modify them for course assignment later, **then clone** your forked codebase to the Pi5 in `~/mbot_ws`.
        
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
        - The calibration script, `mbot_firmware/build/tests/mbot_calibrate_classic.uf2`
        - The MBot firmware, `mbot_firmware/build/src/mbot.uf2`


### 2. Calibrate the MBot and flash the firmware
In this step, we are going to flash the calibration script onto the Pico to calibrate it before we flash the firmware.

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

2. Install the MBot Web App
    1. Download the latest web app release and unpack it
    ```bash
    $ cd ~/mbot_ws
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
