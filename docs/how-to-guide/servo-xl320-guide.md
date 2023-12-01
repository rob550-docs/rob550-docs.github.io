---
layout: default
title: Servo XL320 Guide
parent: How-to Guide
grand_parent: Botlab
nav_order: 2
last_modified_at: 2023-11-16 17:37:48 -0500
---

> This guide introduces how to manuver the XL320 servo using the `mbot_xl320_library` code example provided.

### Contents
* TOC
{:toc}

## Assembly
### Option 1: Using USB to TTL adapter and AX to XL320 adapter
1. To prepare your adapter board for connection, you need to solder a 3-pin right-angle header onto the board, and snip the middle pin. Refer to the first image below for details.
2. Once the header is in place, proceed to connect all the components as illustrated in the second image.
3. Next, attach the power cable to the control board as shown in the third image. We use the power bank to power the control board, and use the servo port to power the two XL320 servos. For clarity, see the fourth image. Note that we are using the SV2 port, make sure to connect the GND pin and the 6V pin accordingly on both ends.
    <div class="popup-gallery">
        <a href="/assets/images/how-to/xl320-guide/xl320_soldering.jpg" title=""><img src="/assets/images/how-to/xl320-guide/xl320_soldering.jpg" width="200" height="200"></a>
        <a href="/assets/images/how-to/xl320-guide/xl320_daisychain.png" title=""><img src="/assets/images/how-to/xl320-guide/xl320_daisychain.png" width="200" height="200"></a>
    </div>
    <div class="popup-gallery">
        <a href="/assets/images/how-to/xl320-guide/xl320_to_board1.jpg" title=" "><img src="/assets/images/how-to/xl320-guide/xl320_to_board1.jpg" width="200" height="200"></a>
        <a href="/assets/images/how-to/xl320-guide/xl320_to_board2.jpg" title=" "><img src="/assets/images/how-to/xl320-guide/xl320_to_board2.jpg" width="200" height="200"></a>
    </div>

### Option 2: Using the custom UART to XL320 board 

| Components               |# | 
|:-----------------------  |:-|
| Rectangular Connectors (6/4/3/1 ) |4 |
| UART to XL320 board               |1 |
| XL320 servo                       |2 |
| Jumper wires (Black/Red/Yellow/Blue/Green/White)   |6| 


1. Assemble the jumper wires with the connectors:
    - 6-Pin Rectangular Connector: Insert the wires in the following order: black, red, green, white, yellow, blue. Refer to the image below for guidance. This color order is not mandatory if you are familiar with circuit wiring, but maintaining consistency across all setups in the class simplifies troubleshooting later.
    - 4-Pin Rectangular Connector:  Connect the black and red wires. Insert the black wire into the rightmost slot. By convention, black is used for GND (ground), and red is used for voltage. This arrangement aligns with the corresponding pin layout on the control board to which it will be connected.
    - 3-Pin Rectangular Connector: Insert the wires in the order of yellow, blue, green.
    - 1-Pin Rectangular Connector: Connect the white wire.

        <a class="image-link" href="/assets/images/how-to/xl320-guide/xl320_uart3.jpg">
        <img src="/assets/images/how-to/xl320-guide/xl320_uart3.jpg" alt=" " style="max-width:300px;"/>
        </a>

2. Connect connectors to UART board, Jetson, and Control board

    | Wire | Pin | Info | 
    |:-----|:--- |:-----|
    | Blue   | RX (Receive) |where Jetson Nano listens for signals |
    | Yellow | TX (Transmit)|where Jetson Nano sends out signals |
    | White  | 3V3              | power the logic circuit|
    | Green  | CTL (Control)    | this pin acts like a switch|
    | Red    | 7V5              | power the servo|
    | Black  | GND              | |

    The table above outlines the relationship between colors and pins.

    Refer to the first image below for guidance on correctly connecting the 6-pin connector to the UART board. Next, attach the 4-pin connector to the control board.
    The third and fourth images show where to connect the 3-pin and 1-pin connectors to the Jetson Nano.

    <div class="popup-gallery">
        <a href="/assets/images/how-to/xl320-guide/xl320_uart2.jpg" title=""><img src="/assets/images/how-to/xl320-guide/xl320_uart2.jpg" width="200" height="200"></a>
        <a href="/assets/images/how-to/xl320-guide/xl320_uart1.jpg" title=""><img src="/assets/images/how-to/xl320-guide/xl320_uart1.jpg" width="200" height="200"></a>
    </div>
    <div class="popup-gallery">
        <a href="/assets/images/how-to/xl320-guide/xl320_uart4.jpg" title=""><img src="/assets/images/how-to/xl320-guide/xl320_uart4.jpg" width="200" height="200"></a>
        <a href="/assets/images/how-to/xl320-guide/xl320_uart6.jpg" title=""><img src="/assets/images/how-to/xl320-guide/xl320_uart6.jpg" width="200" height="200"></a>
    </div>

3. Connect the 2 servos to the UART board

    The result should look like the image below:

    <a class="image-link" href="/assets/images/how-to/xl320-guide/xl320_uart5.jpg">
    <img src="/assets/images/how-to/xl320-guide/xl320_uart5.jpg" alt=" " style="max-width:300px;"/>
    </a>

## Preparation 

For this step, use a USB connection assembled from Option 1 above, and connect your two servos to it. You can find one in the lab and only need to use it for once.

### 1. Download "Dynamixel Wizard 2.0"
Using Dynamixel Wizard, you can change the servo's ID, check the port name, and make many other adjustments.

1. Download [Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) onto your laptop, or use the lab's laptop that already has "Dynamixel Wizard" installed.
2. Then plug the USB into the laptop and open the Dynamixel Wizard. 
    - Click "Scan": the application will begin scanning for any connected servos. This process may take some time. 
    - Click "Options": a window will appear as shown below, where you can deselect some options to speed up the scanning process. For instance, you can narrow the ID range, choose the default baud rate, and select only Protocol 2.0 since the XL320 uses Protocol 2.0.
    <a class="image-link" href="/assets/images/how-to/xl320-guide/dynamixel_wizard1.png">
    <img src="/assets/images/how-to/xl320-guide/dynamixel_wizard1.png" alt=" " style="max-width:400px;"/>
    </a>

### 2. Check your servo's ID  
You will need to specify your servo's ID when using the XL320 library we provide. To check your servo's current ID or change it to your preferred number, you need to use the Dynamixel Wizard.

In the image below, you will notice that there are two servos: one with ID 1 and the other with ID 2. To change these IDs, click on the corresponding servo, and in the right-side panel select your preferred ID, and then click "Save" to apply the new ID. 

If you're unsure which servo is which, you can identify each one by toggling the LED switch on and off. This will allow you to visually confirm which servo you are configuring.

<a class="image-link" href="/assets/images/how-to/xl320-guide/dynamixel_wizard2.png">
<img src="/assets/images/how-to/xl320-guide/dynamixel_wizard2.png" alt=" " style="max-width:400px;"/>
</a>


After successfully confirming your servo's ID, we no longer need Dynamixel Wizard:
- If using the Option 1 connection, disconnect the USB from the laptop and connect it to the Jetson.
- If using the Option 2 connection, reconnect your two servos to the UART connection.

### 3. Check the port name
You will need to specify your port name when using the XL320 library we provide:
- If you are using Option 2 UART conenction, you don't have to check it, it will always be `"/dev/ttyTHS1"`.
- If you are using Option 1 USB conenction, you will have to check it because USB port names are dynamic. So to check it, there are 2 options:

    - **Option 1: Using command line tool `dmesg`**

        When plugged the USB into the Jetson and using VSCode's remote, you can use the command line tool to check:
        ```bash
        $ sudo dmesg | grep tty
        ```
        This command shows kernel messages related to tty devices and filters the output for the string "tty". Look for lines that mention a new device being attached. The device will usually be listed as `/dev/ttyUSB0`, `/dev/ttyACM0`, or similar.

        For instance, the output might include a line similar to:
        ```bash
        [ 8263.926975] cdc_acm 1-9:1.0: ttyACM0: USB ACM device
        ```
        Here, the port name is `/dev/ttyACM0`

    - **Option 2: Using VSCode Extension "Serial Monitor"**

        Download the VSCode Extension "Serial Montior", then open the panel by clicking Terminal -> New Terminal. You should see you have a new tab: Serial Monitor, where you can check detected port name there.

        <a class="image-link" href="/assets/images/how-to/xl320-guide/serial_monitor.png">
        <img src="/assets/images/how-to/xl320-guide/serial_monitor.png" alt=" " style="max-width:400px;"/>
        </a>


### 4. Clone the codebase
Run the following commands to install the xl320 library
```bash
$ cd ~/mbot_ws
$ git clone https://gitlab.eecs.umich.edu/rob550-f23/mbot_xl320_library.git
$ cd mbot_xl320_library
$ ./install.sh
```

## Codebase

Now the library is ready to use. There are two python scripts under `/examples` for reference. 
```bash
$ sudo python3 rotate_in_circle.py
# and
$ sudo python3 rotate_full_range.py
```

To run the examples, you need to modify some of the variables to suit your specific setup:
```python
CONNECTION_DEVICE = "UART"    # change to "UART" if you are using UART connection
#CONNECTION_DEVICE = "USB"   # change to "USB" if you are using USB2AX connection
PORT_NAME = "/dev/ttyTHS1"    # UART has fixed port name ttyTHS1, no need to check the name
#PORT_NAME = "/dev/ttyACM0"  # USB port names are dynamic you need to check what it is 
#...
# defines the servo's ID
servo1_ID = 1
servo2_ID = 2
```


To uninstall, run:
```bash
$ sudo pip3 uninstall -y mbot_xl320_library dynamixel_sdk
```
After uninstalling, if you want to remove the folder, run:
```bash
$ sudo rm -rf mbot_xl320_library
```
- Please do not uninstall it directly by `sudo rm -rf mbot_xl320_library`, this will cause error when you re-install it.

## Troubleshooting
If you have errors saying: 
```
"[RxPacketError] Hardware error occurred. Check the error at Control Table (Hardware Error Status)!"
```
```
"[TxRxResult] There is no status packet!"
```
There could be many reasons behind this issue. 
If everything still functions but you continue to see these errors, try unplugging and then replugging the wires to ensure all connections are secure.