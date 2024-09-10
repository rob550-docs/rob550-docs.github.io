---
layout: default
title: Middle Plate
parent: Assembly Guide
grand_parent: Botlab
nav_order: 2
last_modified_at: 2024-09-09 14:37:48 -0500
---
{: .important}
This guide is for Differential Drive MBot Classic.

## Middle Assembly

1. Mount standoffs on Jetson Nano

    | Components     | #         | 
    |:-------------|:---- -------|
    |Middle Acrylic Sheet     |1|
    | Nvidia Jetson Nano       |1|
    | M2.5 X 30mm Standoffs |4| 
    | M2.5 x 6mm Screws        |4| 

    <a class="image-link" href="/assets/images/botlab/assembly/middlePlate/jetsonstandoff.jpg">
        <img src="/assets/images/botlab/assembly/middlePlate/jetsonstandoff.jpg" alt="" style="max-width:400px;"/>
    </a>

2. Assemble the battery slot

    | Components     | #         | 
    |:-------------|:---- -------|
    | Zip tie                  |4|
    | batter slot pieces |4|

    <div class="popup-gallery">
    <a href="/assets/images/botlab/assembly/middlePlate/battery_slot1.jpg" title="Zip tie the battery slot pieces"><img src="/assets/images/botlab/assembly/middlePlate/battery_slot1.jpg" width="200" height="300"></a>
    <a href="/assets/images/botlab/assembly/middlePlate/battery_slot2.jpg" title="battery slot result"><img src="/assets/images/botlab/assembly/middlePlate/battery_slot2.jpg" width="200" height="300"></a>
    </div>


3. Mount Jetson Nano to the middle plate

    | Components     | #         | 
    |:-------------|:---- -------|
    | M2.5 x 8mm Screws        |4| 

    <a class="image-link" href="/assets/images/botlab/assembly/middlePlate/jeston2plate.jpg">
        <img src="/assets/images/botlab/assembly/middlePlate/jeston2plate.jpg" alt="" style="max-width:400px;"/>
    </a>

4. Put middle plate standoff on

    | Components     | #         | 
    |:-------------|:---- -------|
    |    2.5" Aluminum standoffs|4| 
    |    M2.5 x 8mm Screws  |4| 

    <a class="image-link" href="/assets/images/botlab/assembly/middlePlate/middleplatestandoff.jpg">
        <img src="/assets/images/botlab/assembly/middlePlate/middleplatestandoff.jpg" alt="" style="max-width:400px;"/>
    </a>

5. Attach camera to the Jetson Nano

    | Components     | #         | 
    |:-------------|:---- -------|
    | Arducam V2 |1| 

    > There is a guide from the arducam official website, check it out [here](https://docs.arducam.com/Nvidia-Jetson-Camera/Jetvariety-Camera/Quick-Start-Guide/).

    We also have a guide below illustrating the steps with the MBot itself. Make sure when you insert the cable, it is fully inserted.

    <div class="popup-gallery">
        <a href="/assets/images/botlab/assembly/middlePlate/connectcam1.jpg" title="Connect the camera to jetson 1"><img src="/assets/images/botlab/assembly/middlePlate/connectcam1.jpg" width="200" height="200"></a>
        <a href="/assets/images/botlab/assembly/middlePlate/connectcam2.jpg" title="Connect the camera to jetson 2"><img src="/assets/images/botlab/assembly/middlePlate/connectcam2.jpg" width="200" height="200"></a>
        <a href="/assets/images/botlab/assembly/middlePlate/connectcam3.jpg" title="Connect the camera to jetson 3"><img src="/assets/images/botlab/assembly/middlePlate/connectcam3.jpg" width="200" height="200"></a>
    </div>


6. Attach the camera and camera mount to middle plate

    | Components     | #         | 
    |:-------------|:---- -------|
    | camera mount |1| 
    | transparent spacer for camera|1|
    | M2 x 6mm Screws        |4| 
    | M2.5 x 8mm Screws     |2| 
    | M2.5 threaded inserts |2|

    Note that the OLED mount and the camera mount are both white and look similar, but they are different!  

    1. Insert the threaded inserts into the camera mount, you need soldering iron for this to heat up the inserts.
    2. Attach the camera mount below the middle plate with the screw slot facing upwards, as shown in the image.
    2. Next, attach the camera to the mount and put the transparent spacer between camera and the mount to protect the components on the camera's electronics.

        Note: If you can't find M2 x 6mm screws, M2 x 8mm will also work. However, please be cautious when using the longer screws for the top two holes, as they will stick out and against the middle plate. Avoid overtightening those top two screws.

    <div class="popup-gallery">
        <a href="/assets/images/botlab/assembly/middlePlate/cameramount1.jpg" title="Attach camera mount to plate 1"><img src="/assets/images/botlab/assembly/middlePlate/cameramount1.jpg" width="200" height="200"></a>
        <a href="/assets/images/botlab/assembly/middlePlate/cameramount2.jpg" title="Attach camera mount to plate 2"><img src="/assets/images/botlab/assembly/middlePlate/cameramount2.jpg" width="200" height="200"></a>
   </div>
    <div class="popup-gallery">
        <a href="/assets/images/botlab/assembly/middlePlate/cameramount3.jpg" title="Attach camera to camera mount"><img src="/assets/images/botlab/assembly/middlePlate/cameramount3.jpg" width="200" height="200"></a>
        <a href="/assets/images/botlab/assembly/middlePlate/cameramount4.jpg" title="Attach camera to camera mount"><img src="/assets/images/botlab/assembly/middlePlate/cameramount4.jpg" width="200" height="200"></a>
    </div>

7. Assemble the jumper wires

    | Components     | #         | 
    |:-------------|:-----------|
    | jumper wires (Black/Red/Yellow/Blue/Green/White)   |6| 
    | Crimp connector housings (3/4/6 pins)   |3| 

    You need to build the OLED/Bootload cable assembly which is shown. This cable plugs into the Jetson or RasPi 40pin header, the OLED module, and the Control board and allows both communicating with the OLED and controlling the run and bootload modes on the control board.

    Gather the components listed in the table and ensure you have all the items shown in the image below. Assemble a 3-heads wire as depicted in the second image. **Color and order matters!** Your assembled wire should look exactly like the image indicated.

    <div class="popup-gallery">
    <a href="/assets/images/botlab/assembly/middlePlate/mbot_jumper_wires.jpg" title="Assemble the jumper wires 1"><img src="/assets/images/botlab/assembly/middlePlate/mbot_jumper_wires.jpg" width="200" height="200"></a>
    <a href="/assets/images/botlab/assembly/middlePlate/pi5_jumper_wires_assemble.jpg" title="Assemble the jumper wires 2"><img src="/assets/images/botlab/assembly/middlePlate/pi5_jumper_wires_assemble.jpg" width="400" height="200"></a>
    </div>

8. Connect the assembled jumper wire to the control board and Jetson

    1. Plug the 6PIN connector into the Jetson pins, connecting the cables according to the marks on the board. Here, you should have white cable to pin 11 and the black cable to GND, as illustrated in the image above.
    2. Thread the 3PIN connector through the middle plate like the 2nd image shown.
    3. Insert the 3PIN connector into the control board's RUN/BTLD pins, as shown in the image below.
    <br><br>
    <div class="popup-gallery">
    <a href="/assets/images/botlab/assembly/middlePlate/jetson_connect_jumpwires1.jpg" title="Connect the jumper wires 1"><img src="/assets/images/botlab/assembly/middlePlate/jetson_connect_jumpwires1.jpg" width="200" height="200"></a>
    <a href="/assets/images/botlab/assembly/middlePlate/jetson_connect_jumpwires2.jpg" title="Connect the jumper wires 2"><img src="/assets/images/botlab/assembly/middlePlate/jetson_connect_jumpwires2.jpg" width="200" height="200"></a>
    <a href="/assets/images/botlab/assembly/middlePlate/pi5_connect_jumpwires3.jpg" title="Connect the jumper wires 3"><img src="/assets/images/botlab/assembly/middlePlate/pi5_connect_jumpwires3.jpg" width="200" height="200"></a>
    </div>

9. Attach middle part to bottom part

    | Components     | #         | 
    |:-------------|:---- -------|
    |     4-40 thumb screw 3/8"|4| 

    Use 4 thumb screws to put the middle and bottom parts together.

    <a class="image-link" href="/assets/images/botlab/assembly/middlePlate/middle_bottom.jpg">
    <img src="/assets/images/botlab/assembly/middlePlate/middle_bottom.jpg" alt="" style="max-width:300px;"/>
    </a>    

If you have successfully assembled the middle plate, the result should look like this:

<a class="image-link" href="/assets/images/botlab/assembly/middlePlate/middlefinish.jpg">
    <img src="/assets/images/botlab/assembly/middlePlate/middlefinish.jpg" alt="" style="max-width:300px;"/>
</a>

Now let's move to the top part.


