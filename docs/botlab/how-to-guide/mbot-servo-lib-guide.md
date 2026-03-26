---
layout: default
title: mbot_servo_lib Guide
parent: How-to Guide
grand_parent: Botlab
nav_order: 2
last_modified_at: 2026-03-26 16:37:48 -0500
---

### Contents
* TOC
{:toc}

## Assembly
We use the custom UART to XL320 board here:

| Components               |Number | 
|:-----------------------  |:-|
| Connector Housing (looks like [this](https://www.pololu.com/product/1905)) - 6 pin |1 |
| Connector Housing (looks like [this](https://www.pololu.com/product/1902)) - 3 pin |1 |
| Connector Housing (looks like [this](https://www.pololu.com/product/1901)) - 2 pin |1 |
| Connector Housing (looks like [this](https://www.pololu.com/product/1900)) - 1 pin |1 |
| UART to XL320 board               |1 |
| Servo (XL330 or XL430)            |based on your project |
| Jumper wires (Black/Red/Yellow/Blue/Green/White)   |6| 

### 0. Get all the components
One thing to notice, there are 2 versions of UART to XL320 boards, show in the image below. Always choose the newer version:

<a class="image-link" href="/assets/images/botlab/how-to/mbot-servo-guide/new_old_boards.png">
<img src="/assets/images/botlab/how-to/mbot-servo-guide/new_old_boards.png" alt=" " style="max-width:300px;"/>
</a>

### 1. Assemble the jumper wires with the connector housings:
1. **6-Pin housing**: Insert the wires in the following order: **black, red, green, white, yellow, blue**. 
    - Refer to the image below. This color order is not mandatory if you are familiar with circuit wiring, but maintaining consistency across all setups in the class simplifies troubleshooting later.
2. **3-Pin housing**: Insert the wires in the order of **yellow, blue, green**.
3. **2-Pin housing**:  Connect the **black and red** wires.
    - For this one, you can use any of 2-pin, 3-pin, or 4-pin housing, just remember to put black on the side, and red wire next to it. This will later connect to the servo pin header on the control board. The header itself has four pins arranged together, so any of these housing options (2/3/4 pin) will fit and function correctly.
4. **1-Pin housing**: Connect the **white** wire.

    <a class="image-link" href="/assets/images/botlab/how-to/mbot-servo-guide/xl320_uart3.jpg">
    <img src="/assets/images/botlab/how-to/mbot-servo-guide/xl320_uart3.jpg" alt=" " style="max-width:300px;"/>
    </a>

### 2. Connect UART board, Pi 5, and Control board

| Wire | Pin | Info | 
|:-----|:--- |:-----|
| Blue   | RX (Receive) |where Pi 5 listens for signals |
| Yellow | TX (Transmit)|where Pi 5 sends out signals |
| White  | 3V3              | power the logic circuit|
| Green  | CTL (Control)    | this pin acts like a switch|
| Red    | 7V5              | power the servo|
| Black  | GND              | |

The table above outlines the relationship between colors and pins.

1. Refer to the first image below to connect the 6-pin connector to the UART board. 
2. The second image, attach the 2-pin connector to the control board. You can choose any of the `SV` slots. Red to 6V, black to GND.
3. The third image, show where to connect the 3-pin to the Pi 5.
4. The fourth image, show where to connect th 1-pin connector, PIN 17 (3V3 Power)

<div class="popup-gallery">
    <a href="/assets/images/botlab/how-to/mbot-servo-guide/xl320_uart2.jpg" title=""><img src="/assets/images/botlab/how-to/mbot-servo-guide/xl320_uart2.jpg" width="300" height="200"></a>
    <a href="/assets/images/botlab/how-to/mbot-servo-guide/xl320_uart1.jpg" title=""><img src="/assets/images/botlab/how-to/mbot-servo-guide/xl320_uart1.jpg" width="300" height="200"></a>
</div>
<div class="popup-gallery">
    <a href="/assets/images/botlab/how-to/mbot-servo-guide/xl320_uart4.jpg" title=""><img src="/assets/images/botlab/how-to/mbot-servo-guide/xl320_uart4.jpg" width="300" height="200"></a>
    <a href="/assets/images/botlab/how-to/mbot-servo-guide/xl320_uart6.jpg" title=""><img src="/assets/images/botlab/how-to/mbot-servo-guide/xl320_uart6.jpg" width="300" height="200"></a>
</div>

<a class="image-link" href="https://pinout-ai.s3.eu-west-2.amazonaws.com/raspberry-pi-5-gpio-pinout-diagram.webp">
<img src="https://pinout-ai.s3.eu-west-2.amazonaws.com/raspberry-pi-5-gpio-pinout-diagram.webp" alt=" " style="max-width:600px;"/>
</a>

### 3. Connect the servo to the UART board

The connection should looks like the image below:

<a class="image-link" href="/assets/images/botlab/how-to/mbot-servo-guide/uart2servo.png">
<img src="/assets/images/botlab/how-to/mbot-servo-guide/uart2servo.png" alt=" " style="max-width:300px;"/>
</a>

## System Config
1. Edit the Configuration File.
    ```bash
    sudo nano /boot/firmware/config.txt
    ```
2. Add the following lines to the end of the file to enable the primary UART on GPIO pins 14 (TX) and 15 (RX). Save the file by ctrl+X then Enter.
    ```bash
    [pi5]
    dtoverlay=uart0-pi5
    ```
3. Run the following to add user to dialout group:
    ```bash
    sudo usermod -a -G dialout $USER
    ```
4. Reboot your Raspberry Pi for the changes to take effect.
    ```bash
    sudo reboot
    ```
5. Test. Run the following command we should see `ttyAMA0` list
    ```
    $ ls /dev | grep ttyAMA
    ttyAMA0
    ttyAMA10
    ```
    Then run `groups` and you should see `dialout` listed.


## Install
Go to the class GitLab page you should see the repository `mbot_servo_lib`. Clone it to the home directory.

Run the following install script
```bash
cd ~/mbot_servo_lib
./install.sh
```
- After installing the library, any script, anywhere, can do `from mbot_servo_library import *`

### How to use
1. Check the servo ID first. This program will ping all the possible IDs and give you the ID it detects.
    ```bash
    cd ~/mbot_servo_lib/tests
    python3 check_servo_id.py
    ```
    - **Connect only one servo at a time to check its ID and change it if you find duplicates.**
2. If you use more than 1 servo, and there are duplicate IDs, connect only one of the affected servos to the board (no other servos should be connected), then modify `change_servo_id.py` to assign it a new ID.
    ```python
    CURRENT_ID = 1  # The ID you want to change
    NEW_ID = 3      # The new ID you want to set
    ```
    Then run:
    ```bash
    python3 change_servo_id.py
    ```
3. We have 3 exmaples
    - `tests/position_control.py` - This example shows position control. The servo moves to a target position and stops. Position is a count from 0 to 4095, representing one full rotation (360°). The servo will never go beyond that range.
    - `tests/extended_position_control.py` - Same idea as position mode, but the count is linear and never resets — it just keeps going up or down across rotations. 4096 = 1 full turn, 8192 = 2 turns, -4096 = 1 turn backwards. Supports up to ±256 full rotations. 
    - `tests/velocity_control.py` - This example shows velocity control. The servo spins like a wheel — there is no target position. You set a speed (as a percentage) and a direction (CW or CCW), and it keeps spinning until you tell it to stop.
    
    You need to define the servo ID to use the examples:
    ```python
    # define the servo's ID
    servo_ID = 1
    ```
    Then run:
    ```bash
    cd ~/mbot_servo_lib/tests
    python3 position_control.py
    ```

    Demo video shows the expected result from the 3 example:

    <iframe width="460" height="260" src="https://www.youtube.com/embed/gJAQZfpep1g?si=uioESTtm9Hpm2wFK" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Troubleshooting
If you have errors saying: 
```
"[RxPacketError] Hardware error occurred. Check the error at Control Table (Hardware Error Status)!"
```
Run the reboot script to reboot the servo, or unplug and then replug the servo to the board.
```bash
cd ~/mbot_servo_lib/tests
# change the servo ID in reboot.py
python3 reboot.py
```

## Uninstall
```bash
sudo pip3 uninstall dynamixel_sdk mbot_servo_library --break-system-packages
```
After uninstalling, if you want to remove the folder, run:
```bash
sudo rm -rf mbot_servo_library
```
- Please **DO NOT** uninstall it directly by `sudo rm -rf mbot_servo_library`, this will cause error when you re-install it.


## Legacy
> This section is irrelavant to 2026 ROS2 MBot!!!

### 0. Using USB to TTL adapter and AX to XL320 adapter
1. To prepare your adapter board for connection, you need to solder a 3-pin right-angle header onto the board, and snip the middle pin. Refer to the first image below for details.
2. Once the header is in place, proceed to connect all the components as illustrated in the second image.
3. Next, attach the power cable to the control board as shown in the third image. We use the power bank to power the control board, and use the servo port to power the two XL320 servos. For clarity, see the fourth image. Note that we are using the SV2 port, make sure to connect the GND pin and the 6V pin accordingly on both ends.
    <div class="popup-gallery">
        <a href="/assets/images/botlab/how-to/mbot-servo-guide/xl320_soldering.jpg" title=""><img src="/assets/images/botlab/how-to/mbot-servo-guide/xl320_soldering.jpg" width="200" height="200"></a>
        <a href="/assets/images/botlab/how-to/mbot-servo-guide/xl320_daisychain.png" title=""><img src="/assets/images/botlab/how-to/mbot-servo-guide/xl320_daisychain.png" width="200" height="200"></a>
    </div>
    <div class="popup-gallery">
        <a href="/assets/images/botlab/how-to/mbot-servo-guide/xl320_to_board1.jpg" title=" "><img src="/assets/images/botlab/how-to/mbot-servo-guide/xl320_to_board1.jpg" width="200" height="200"></a>
        <a href="/assets/images/botlab/how-to/mbot-servo-guide/xl320_to_board2.jpg" title=" "><img src="/assets/images/botlab/how-to/mbot-servo-guide/xl320_to_board2.jpg" width="200" height="200"></a>
    </div>

### 1. Download "Dynamixel Wizard 2.0"
Using Dynamixel Wizard, you can change the servo's ID, check the port name, and make many other adjustments.

1. Download [Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) onto your laptop, or use the lab's laptop that already has "Dynamixel Wizard" installed.
2. Then plug the USB into the laptop and open the Dynamixel Wizard. 
    - Click "Scan": the application will begin scanning for any connected servos. This process may take some time. 
    - Click "Options": a window will appear as shown below, where you can deselect some options to speed up the scanning process. For instance, you can narrow the ID range, choose the default baud rate, and select only Protocol 2.0 since the XL320 uses Protocol 2.0.
    <a class="image-link" href="/assets/images/botlab/how-to/mbot-servo-guide/dynamixel_wizard1.png">
    <img src="/assets/images/botlab/how-to/mbot-servo-guide/dynamixel_wizard1.png" alt=" " style="max-width:400px;"/>
    </a>

### 2. Check your servo's ID  
You will need to specify your servo's ID when using the XL320 library we provide. To check your servo's current ID or change it to your preferred number, you need to use the Dynamixel Wizard.

In the image below, you will notice that there are two servos: one with ID 1 and the other with ID 2. To change these IDs, click on the corresponding servo, and in the right-side panel select your preferred ID, and then click "Save" to apply the new ID. 

If you're unsure which servo is which, you can identify each one by toggling the LED switch on and off. This will allow you to visually confirm which servo you are configuring.

<a class="image-link" href="/assets/images/botlab/how-to/mbot-servo-guide/dynamixel_wizard2.png">
<img src="/assets/images/botlab/how-to/mbot-servo-guide/dynamixel_wizard2.png" alt=" " style="max-width:400px;"/>
</a>


After successfully confirming your servo's ID, we no longer need Dynamixel Wizard. Disconnect the USB from the laptop and connect it to the Pi.

### 3. Check the port name
- **Option 1: Using command line tool `dmesg`**

    When plugged the USB into the Pi and using VSCode's remote, you can use the command line tool to check:
    ```bash
    sudo dmesg | grep tty
    ```
    This command shows kernel messages related to tty devices and filters the output for the string "tty". Look for lines that mention a new device being attached. The device will usually be listed as `/dev/ttyUSB0`, `/dev/ttyACM0`, or similar.

    For instance, the output might include a line similar to:
    ```bash
    [ 8263.926975] cdc_acm 1-9:1.0: ttyACM0: USB ACM device
    ```
    Here, the port name is `/dev/ttyACM0`

- **Option 2: Using VSCode Extension "Serial Monitor"**

    Download the VSCode Extension "Serial Montior", then open the panel by clicking Terminal -> New Terminal. You should see you have a new tab: Serial Monitor, where you can check detected port name there.

    <a class="image-link" href="/assets/images/botlab/how-to/mbot-servo-guide/serial_monitor.png">
    <img src="/assets/images/botlab/how-to/mbot-servo-guide/serial_monitor.png" alt=" " style="max-width:400px;"/>
    </a>

To run the examples, you need to modify some of the variables to suit your specific setup:
```python
CONNECTION_DEVICE = "UART"    # change to "UART" if you are using UART connection
#CONNECTION_DEVICE = "USB"   # change to "USB" if you are using USB2AX connection
PORT_NAME = "/dev/ttyACM0"    # UART has fixed port name on Pi, no need to check the name
#PORT_NAME = "/dev/ttyACM0"  # USB port names are dynamic you need to check what it is 
#...
# defines the servo's ID
servo1_ID = 1
servo2_ID = 2
```
