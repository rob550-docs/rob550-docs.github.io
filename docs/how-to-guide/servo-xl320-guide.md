---
layout: default
title: Servo XL320 Guide
parent: How-to Guide
nav_order: 2
last_modified_at: 2023-11-07 18:37:48 -0500
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

### Option 2: Using the custom PCB
Professor Gaskell has ordered some custom PCBs that should allow you to plug the Dynamixel XL320s straight into the MBot Control board or the UART pins on the Jetson 40-pin header. 

The actual PCBs are still under testing, but their design is as shown in the image provided below.

<a class="image-link" href="/assets/images/how-to/xl320-guide/xl320_pcb.png">
<img src="/assets/images/how-to/xl320-guide/xl320_pcb.png" alt=" " style="max-width:270px;"/>
</a>

## Codebase
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


**After successfully confirming your servo's ID, unplug the USB from the laptop and connect it to the Jetson.**

### 3. Check the port name
You will need to specify your port name when using the XL320 library we provide.

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

    Download the Extension "Serial Montior", then open the panel by clicking Terminal -> New Terminal. You should see you have a new tab: Serial Monitor terminal, you can check detected port name there.

    <a class="image-link" href="/assets/images/how-to/xl320-guide/serial_monitor.png">
    <img src="/assets/images/how-to/xl320-guide/serial_monitor.png" alt=" " style="max-width:400px;"/>
    </a>


### 4. Clone the codebase
```bash
$ cd mbot_ws
$ git clone [placeholder]
```
this is under editting...