---
layout: default
title: Mbot Assembly
parent: Getting Started
nav_order: 1
last_modified_at: 2023-09-15 17:37:48 -0500
---


> This guide will walk you through the steps needed to assemble the MBot Classic. 

**TODO: Insert a finished mbot here**

### Contents
* TOC
{:toc}

## Bottom Assembly
1. Mount the caster to the bottom

    | Components               |# | 
    |:-----------------------  |:-|
    | Metal ball               |1 |
    | Bottom Acrylic Sheet     |1 |
    | 1/8" spacer              |1 |
    | M2 hex nuts              |2 |
    | M2 x 12mm screw          |2 | 
    | Caster housing           |1 |

    Collect all the necessary components and assemble them as illustrated in the images below.

    <div class="popup-gallery">
        <a href="/assets/images/assembly/bottomPlate/caster1.jpg" title="Mount caster 1"><img src="/assets/images/assembly/bottomPlate/caster1.jpg" width="200" height="200"></a>
        <a href="/assets/images/assembly/bottomPlate/caster2.jpg" title="Mount caster 2"><img src="/assets/images/assembly/bottomPlate/caster2.jpg" width="200" height="200"></a>
        <a href="/assets/images/assembly/bottomPlate/caster3.jpg" title="Mount caster 3"><img src="/assets/images/assembly/bottomPlate/caster3.jpg" width="200" height="200"></a>
    </div>


2. Mount the motor brackets

    | Components     | #         | 
    |:-------------|:---- -------|
    | M2.5 x 8mm Screws        |8| 
    | Motor Mount (3D printed) |2| 

    Collect all the necessary components and assemble them as illustrated in the images below. When attaching the mount to the bottom plate, ensure that the slot faces inward.

    <div class="popup-gallery">
        <a href="/assets/images/assembly/bottomPlate/bracketmount1.jpg" title="Mount motor brackets 1"><img src="/assets/images/assembly/bottomPlate/bracketmount1.jpg" width="200" height="200"></a>
        <a href="/assets/images/assembly/bottomPlate/bracketmount2.jpg" title="Mount motor brackets 2"><img src="/assets/images/assembly/bottomPlate/bracketmount2.jpg" width="200" height="200"></a>
        <a href="/assets/images/assembly/bottomPlate/bracketmount3.jpg" title="Mount motor brackets 3"><img src="/assets/images/assembly/bottomPlate/bracketmount3.jpg" width="200" height="200"></a>
    </div>

3. Mount the motors

    | Components     | #         | 
    |:-------------|:---- -------|
    | M2.5 x 6mm Screws        |4| 
    | Gear Motors w/ Wiring & Encoders |2| 
    
    Collect all the necessary components and assemble them as illustrated in the images below. Note that you need to align unthreaded holes in motor vertically to the holes in the motor mount, while the wires should face the back of the MBot as shown in the second figure.

    <div class="popup-gallery">
        <a href="/assets/images/assembly/bottomPlate/motormount1.jpg" title="Mount motor 1"><img src="/assets/images/assembly/bottomPlate/motormount1.jpg" width="200" height="200"></a>
        <a href="/assets/images/assembly/bottomPlate/motormount2.jpg" title="Mount motor 2"><img src="/assets/images/assembly/bottomPlate/motormount2.jpg" width="200" height="200"></a>
        <a href="/assets/images/assembly/bottomPlate/motormount3.jpg" title="Mount motor 3"><img src="/assets/images/assembly/bottomPlate/motormount3.jpg" width="200" height="200"></a>
    </div>

4. Mount the tall 1.5” aluminum standoffs

    | Components               |# | 
    |:-----------------------  |:-|
    | 1.5" Aluminum standoffs  |4 |
    | 4-40 machine screw 3/8"  |4 | 
 
    Gather all the components and assemble as shown in the images below.

    <div class="popup-gallery">
        <a href="/assets/images/assembly/bottomPlate/aluminumstandoffs1.jpg" title="Mount aluminum standoffs 1"><img src="/assets/images/assembly/bottomPlate/aluminumstandoffs1.jpg" width="200" height="200"></a>
        <a href="/assets/images/assembly/bottomPlate/aluminumstandoffs2.jpg" title="Mount aluminum standoffs 2"><img src="/assets/images/assembly/bottomPlate/aluminumstandoffs2.jpg" width="200" height="200"></a>
    </div>

5. Put the wheels on

    | Components               |# | 
    |:-----------------------  |:-|
    | Scooter Wheels 84 X 24 mm  |2 |
    | Hex Key  |1 | 

    1. If your wheels are not assembled yet, you need to assemble it first

        Follow the steps below: first, press fit the machined wheel adaptor into one side of the wheels, ensuring a part of it sticks out. Then press fit the flat metal piece into the opposite side, aligning the three holes. Finally, insert the M3x20mm screws to secure the parts together, as demonstrated in the second step. The third and fourth steps show how to screw in the wheel adapter using the hex wrench.

        <a class="image-link" href="/assets/images/assembly/bottomPlate/assemblewheel.jpg">
            <img src="/assets/images/assembly/bottomPlate/assemblewheel.jpg" alt="" style="max-width:400px;"/>
        </a>

    2. If your wheels are assembled already

        Then you can directly put the wheel on, use the hex key to tight the wheels. Note that it is necessary to leave a nominal space between the wheel and the screw head of the motor, as depicted in the image below. 

        <div class="popup-gallery">
            <a href="/assets/images/assembly/bottomPlate/wheelon1.jpg" title="Mount the wheel on 1"><img src="/assets/images/assembly/bottomPlate/wheelon1.jpg" width="300" height="300"></a>
            <a href="/assets/images/assembly/bottomPlate/wheelon2.jpg" title="Mount the wheel on 2"><img src="/assets/images/assembly/bottomPlate/wheelon2.jpg" width="200" height="200"></a>
        </div>

6. Assemble the MBot board standoffs

    | Components     | #         | 
    |:-------------|:---- -------|
    | MBot board               |1|
    | M2.5 8mm Nylon Standoffs |4| 
    | M2.5 x 6mm Screws        |4| 

    Gather all the components you need and assemble them as depicted in the image below. 

    <div class="popup-gallery">
        <a href="/assets/images/assembly/bottomPlate/mbotboard1.jpg" title="Assemble the MBot board standoffs 1"><img src="/assets/images/assembly/bottomPlate/mbotboard1.jpg" width="300" height="300"></a>
        <a href="/assets/images/assembly/bottomPlate/mbotboard2.jpg" title="Assemble the MBot board standoffs 2"><img src="/assets/images/assembly/bottomPlate/mbotboard2.jpg" width="300" height="300"></a>
    </div>

7. Connect the wires to MBot Board

    1. Arrange the wires through the bottom plate as shown in the image below.
    2. Examine the pins. In the case illustrated in the image, both the left and right motor have the green wire connected to the GND pin. We will use this info to connect to the MBot Board.
    3. Identify the GND pin on the MBot Board and connect the wires to it accordingly. Ensure the green wire is connected to the GND pin.


    <div class="popup-gallery">
        <a href="/assets/images/assembly/bottomPlate/connectwire1.jpg" title="Connect the wires to MBot Board 1"><img src="/assets/images/assembly/bottomPlate/connectwire1.jpg" width="200" height="200"></a>
        <a href="/assets/images/assembly/bottomPlate/connectwire2.jpg" title="Connect the wires to MBot Board 2"><img src="/assets/images/assembly/bottomPlate/connectwire2.jpg" width="400" height="400"></a>
        <a href="/assets/images/assembly/bottomPlate/connectwire3.jpg" title="Connect the wires to MBot Board 3"><img src="/assets/images/assembly/bottomPlate/connectwire3.jpg" width="200" height="200"></a>
    </div>


8. Final step: attach the MBot Board to the bottom plate

    <a class="image-link" href="/assets/images/assembly/bottomPlate/attachboard.jpg">
        <img src="/assets/images/assembly/bottomPlate/attachboard.jpg" alt="" style="max-width:400px;"/>
    </a>


If you have successfully assembled the bottom plate, the result should look like this:
 
<a class="image-link" href="/assets/images/assembly/bottomPlate/bottom_finish.jpg">
  <img src="/assets/images/assembly/bottomPlate/bottom_finish.jpg" alt="" style="max-width:400px;"/>
</a>

Now let's move to the middle part.

## Middle Assembly

1. Mount standoffs on Jetson Nano

    | Components     | #         | 
    |:-------------|:---- -------|
    |Middle Acrylic Sheet     |1|
    | Nvidia Jetson Nano       |1|
    | M2.5 X 30mm Aluminum Standoffs |4| 
    | M2.5 x 6mm Screws        |4| 

    <a class="image-link" href="/assets/images/assembly/middlePlate/jetsonstandoff.jpg">
        <img src="/assets/images/assembly/middlePlate/jetsonstandoff.jpg" alt="" style="max-width:400px;"/>
    </a>

2. Mount Jetson Nano to the middle plate

    | Components     | #         | 
    |:-------------|:---- -------|
    | M2.5 x 8mm Screws        |4| 

    <a class="image-link" href="/assets/images/assembly/middlePlate/jeston2plate.jpg">
        <img src="/assets/images/assembly/middlePlate/jeston2plate.jpg" alt="" style="max-width:400px;"/>
    </a>

3. Put middle plate standoff on

    | Components     | #         | 
    |:-------------|:---- -------|
    |    2.5" Aluminum standoffs|4| 
    |    M2 x 12mm Screws  |4| 

    <a class="image-link" href="/assets/images/assembly/middlePlate/middleplatestandoff.jpg">
        <img src="/assets/images/assembly/middlePlate/middleplatestandoff.jpg" alt="" style="max-width:400px;"/>
    </a>

4. Attach camera to the Jetson Nano

    | Components     | #         | 
    |:-------------|:---- -------|
    | Arducam V2 |1| 

    > There is a guide from the arducam official website, check it out [here](https://docs.arducam.com/Nvidia-Jetson-Camera/Jetvariety-Camera/Quick-Start-Guide/).

    We also have a guide below illustrating the steps with the MBot itself. Make sure when you insert the cable, it is fully inserted.

    <div class="popup-gallery">
        <a href="/assets/images/assembly/middlePlate/connectcam1.jpg" title="Connect the camera to jetson 1"><img src="/assets/images/assembly/middlePlate/connectcam1.jpg" width="200" height="200"></a>
        <a href="/assets/images/assembly/middlePlate/connectcam2.jpg" title="Connect the camera to jetson 2"><img src="/assets/images/assembly/middlePlate/connectcam2.jpg" width="200" height="200"></a>
        <a href="/assets/images/assembly/middlePlate/connectcam3.jpg" title="Connect the camera to jetson 3"><img src="/assets/images/assembly/middlePlate/connectcam3.jpg" width="200" height="200"></a>
    </div>


5. Attach the camera with the camera mount

    | Components     | #         | 
    |:-------------|:---- -------|
    | camera mount |1| 
    | M2 x 8mm Screws        |4| 
    | M2 Nuts     |4| 

    Align the four holes on both the camera and the camera mount, using four sets of screws and nuts to tighten them. Arrange the cable as shown in the image, and pay attention to which side is the front or back.

    <a class="image-link" href="/assets/images/assembly/middlePlate/cameramount.jpg">
    <img src="/assets/images/assembly/middlePlate/cameramount.jpg" alt="" style="max-width:400px;"/>
    </a>

6. Attach the camera/camera mount to the middle plate

    | Components     | #         | 
    |:-------------|:---- -------|
    | M2.5 x 12mm Screws        |2| 
    | M2.5 Nuts      |2| 

    Align the two holes on both the camera mount and the middle plate, using two sets of screws and nuts to tighten them.

    <a class="image-link" href="/assets/images/assembly/middlePlate/cam2plate.jpg">
    <img src="/assets/images/assembly/middlePlate/cam2plate.jpg" alt="" style="max-width:400px;"/>
    </a>

If you have successfully assembled the middle plate, the result should look like this:

<a class="image-link" href="/assets/images/assembly/middlePlate/middlefinish.jpg">
    <img src="/assets/images/assembly/middlePlate/middlefinish.jpg" alt="" style="max-width:400px;"/>
</a>

Now let's move to the top part.


## Top Assembly

1. Attach the RPLidar standoffs to the Lidar

    | Components     | #         | 
    |:-------------|:---- -------|
    | RPLidar A1 |1| 
    | M2.5 standoffs        |4| 
    | M2.5 Screws   |4|
    
    Attach four standoffs to the Lidar as shown in the image below.

    <div class="popup-gallery">
        <a href="/assets/images/assembly/topPlate/lidarstandoff1.jpg" title="Attach lidar standoff 1"><img src="/assets/images/assembly/topPlate/lidarstandoff1.jpg" width="225" height="225"></a>
        <a href="/assets/images/assembly/topPlate/lidarstandoff2.jpg" title="Attach lidar standoff 2"><img src="/assets/images/assembly/topPlate/lidarstandoff2.jpg" width="200" height="200"></a>
    </div>


2. Attach the RPLidar USB Interface to the underside of the top plate

    | Components     | #         | 
    |:-------------|:---- -------|
    | M2 x 8mm Screws        |2| 
    | M2 Nuts     |2|
    
    Begin by unplugging the connector from the interface. Then, pass it through one of the square holes on the board and re-plug the connector back in. Secure the USB Interface to the top plate using two sets of M2 screws and nuts.


    <div class="popup-gallery">
        <a href="/assets/images/assembly/topPlate/usb1.jpg" title="Attach USB Interface to plate 1"><img src="/assets/images/assembly/topPlate/usb1.jpg" width="200" height="200"></a>
        <a href="/assets/images/assembly/topPlate/usb2.jpg" title="Attach USB Interface to plate 2"><img src="/assets/images/assembly/topPlate/usb2.jpg" width="200" height="200"></a>
        <a href="/assets/images/assembly/topPlate/usb3.jpg" title="Attach USB Interface to plate 3"><img src="/assets/images/assembly/topPlate/usb3.jpg" width="200" height="200"></a>
        <a href="/assets/images/assembly/topPlate/attachusb.jpg" title="Attach USB Interface to plate 4"><img src="/assets/images/assembly/topPlate/attachusb.jpg" width="380" ></a>
    </div>

3. Attach the RPLidar to the upper side of the top plate

    | Components     | #         | 
    |:-------------|:---- -------|
    | M2.5 Screws   |4|

    <div class="popup-gallery">
        <a href="/assets/images/assembly/topPlate/attachlidar1.jpg" title="Attach lidar to plate 1"><img src="/assets/images/assembly/topPlate/attachlidar1.jpg" width="200" height="200"></a>
        <a href="/assets/images/assembly/topPlate/attachlidar2.jpg" title="Attach lidar to plate 2"><img src="/assets/images/assembly/topPlate/attachlidar2.jpg" width="200" height="200"></a>
    </div>

4. Plug in the USB cord to the RPLidar USB Interface, and fastening it with a zip tie

    | Components     | #         | 
    |:-------------|:---- -------|
    | 9" USB Micro Cable  |1| 
    | zip tie   |1|

    <div class="popup-gallery">
        <a href="/assets/images/assembly/topPlate/plugusb1.jpg" title="Plug in USB 1"><img src="/assets/images/assembly/topPlate/plugusb1.jpg" width="225" height="225"></a>
        <a href="/assets/images/assembly/topPlate/plugusb2.jpg" title="Plug in USB 2"><img src="/assets/images/assembly/topPlate/plugusb2.jpg" width="200" height="200"></a>
    </div>

If you have successfully assembled the top plate, the result should look like this:

<a class="image-link" href="/assets/images/assembly/topPlate/topfinish.jpg">
    <img src="/assets/images/assembly/topPlate/topfinish.jpg" alt="" style="max-width:400px;"/>
</a> 

Now let's put all 3 parts together!


## Final Assembly

1. Attach all three parts together

    | Components     | #         | 
    |:-------------|:---- -------|
    |     4-40 thumb screw 3/8"|8| 

    <div class="popup-gallery">
        <a href="/assets/images/assembly/final/final1.jpg" title="Final assemble 1"><img src="/assets/images/assembly/final/final1.jpg" width="200" height="200"></a>
        <a href="/assets/images/assembly/final/final2.jpg" title="Final assemble 2"><img src="/assets/images/assembly/final/final2.jpg" width="200" height="200"></a>
    </div>

2. Connect all the cables and power

    | Components     | #         | 
    |:-------------|:---- -------|
    |  DC power “Y” cable |1| 
    |  12V Power Bank |1| 
    |  9" USB-C Cable |1| 


Yay! Now you have a complete version of MBot!
