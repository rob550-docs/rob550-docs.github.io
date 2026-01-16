---
layout: default
title: Dev Setup for Pi5 Classic
parent: Legacy Guides
nav_order: 6
last_modified_at: 2024-07-12 10:37:48 -0500
---

> This is for Pi5 Mbot Classic Developers to set up from scratch. For any mbot repository, **the dev branch will always have the latest code for development** (when it exists).

# 1. Prepare Your Raspberry Pi
You will need:
- SD card reader
- Laptop

Download Raspberry Pi Imager from [Raspberry Pi Official Website](https://www.raspberrypi.com/software/).

Then open the Imager, select the options as following:
- Device: Raspberry Pi 5
- OS: Raspberry Pi OS (64-bit) - Bookworm
- Storage: Your SD card

Click NEXT; no need to edit settings.

Wait until the process is done, then take out the SD card.

# 2. Customize Your Raspberry Pi
You will need:
- External Monitor
- Micro HDMI cable
- Mouse and Keyboard

## Basic Setup
1. Insert the SD card into the Pi, connect the Pi to an external monitor, and turn on the power.
2. Once the UI appears, follow the setup steps. 
    - username: mbot
    - password: i<3robots!
    - select Firefox as the default browser
    - skip the Wi-Fi setup and software update parts.
3. Once you get to the desktop, connect to the "MSetup" network. Open Firefox and go to the "log in to network" page to reach the UM WiFi configuration page. Click "Laptop, Phone, or Tablet", select "Linux", and click "Join Now". The file "SecureW2_JoinNow.run" will download automatically.
4. Open a terminal, navigate to the file "SecureW2_JoinNow.run", run `chmod +x SecureW2_JoinNow.run` to give it permission, then run `./SecureW2_JoinNow.run`. Enter your credentials to connect to the Wi-Fi.

Congrats, you have discovered the Internet!

## Remote Access Setup
1. Run:
    ```bash
    sudo apt update
    sudo apt upgrade
    ```
2. Download [Visual Studio Code on Raspberry Pi OS](https://code.visualstudio.com/docs/setup/raspberry-pi) by running:
    ```bash
    sudo apt install code
    ```
3. Open Firefox, log in to GitHub, then open a terminal run:
    ```bash
    cd ~
    mkdir workspace
    cd workspace
    # git clone mbot_sys_utils here
    cd mbot_sys_utils
    # dev branch has the latest version of the code
    git checkout dev
    cd install_scripts
    ./install_nomachine.sh
    ```
4. Run `sudo raspi-config`:
    - Interface Options -> SSH -> Enable
    - Advanced Options -> Wayland -> X11 (important for NoMachine) -> Reboot

Congrats, you have discovered remote access and remote desktop! An external monitor is no longer required. Check your IP address using `ip address`:
- SSH into the Pi: `ssh mbot@pi_ip`
- Connect to the desktop using NoMachine

## System Setup
1. Open `mbot_sys_utils`, find `rpi5_config.txt`, and adjust settings as needed.
2. Copy `rpi5_config.txt` to `/boot/firmware`:
    ```bash
    sudo cp rpi5_config.txt /boot/firmware
    ```
3. Modify `config.txt` to include your customized `rpi5_config.txt`:
    ```bash
    sudo nano /boot/firmware/config.txt
    ```
    Then add the following line under the [all] section:
    ```bash
    [all]
    include rpi5_config.txt
    ```
    - Check details at [config.txt - Raspberry Pi Documentation](https://www.raspberrypi.com/documentation/computers/config_txt.html)
5. Copy `mbot_config.txt` to `/boot/firmware`:
```bash
sudo cp mbot_config.txt /boot/firmware
```
6. Edit the configuration:
```bash
sudo nano /boot/firmware/mbot_config.txt
```

# 3. Set Up Your Development Environment
## Install Packages and Services
### mbot_sys_utils
1. Install dependencies:
```bash
cd ~/workspace/mbot_sys_utils
git checkout dev
sudo ./install_scripts/install_mbot_dependencies.sh
./install_scripts/install_lcm.sh
```
2. Install udev rules:
```bash
cd udev_rules
./install_rules.sh
```
3. Install services:
```bash
cd services
./install_mbot_services.sh
```

### mbot_firmware and mbot_lcm_base
1. Clone `mbot_firmware` and `mbot_lcm_base` repositories.
2. Install LCM related components:
```bash
cd ~/workspace/mbot_lcm_base
git checkout dev
./scripts/install.sh
```
    - Now you have mbot system cli tools available.
3. Run the firmware setup script:
```bash
cd ~/workspace/mbot_firmware
git checkout dev
./setup.sh
```
4. Build the firmware:
```bash
cd ~/workspace/mbot_firmware
mkdir build
cd build
cmake ..
make
```
5. Calibrate your mbot (**place the mbot on the floor first**):
```bash
cd ~/workspace/mbot_firmware
sudo ./upload.sh flash build/tests/mbot_calibrate_classic.uf2
```
6. Flash the firmware to the control board:
```bash
sudo ./upload.sh flash build/src/mbot_classic.uf2
```
7. Test the setup
```bash
python3 python/mbot_move_simple.py
```
- Warning: The mbot will move forward quickly with this script. Modify as needed.

### rplidar_lcm_driver and mbot_web_app
1. Clone rplidar_lcm_driver:
```bash
cd ~/workspace/rplidar_lcm_driver/
./scripts/install.sh
```
2. Clone mbot_web_app:
- If no `dev` branch there, use the latest version.
- Follow the setup instructions in the README.md.

### mbot_autonomy
1. Clone mbot_autonomy
2. Install:
```bash
cd ~/workspace/mbot_autonomy
git checkout dev
./scripts/install.sh
```
- Follow the README.md if instructions differ.
