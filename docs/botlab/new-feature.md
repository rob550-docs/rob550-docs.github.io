---
layout: default
title: New Feature!
parent: Botlab
nav_order: 6
last_modified_at: 2024-11-07 12:29:48 -0500
---

We have an upgrade to the OLED screen functionality. Here’s what’s new:
1. The IP address will always appear on the bottom line not cutting in half for Webapp page.
2. The second page now displays real-time battery voltage. If the voltage drops below 9V, the screen will flash as a low-battery warning, reminding you to change the battery.

Follow these steps to update your OLED screen
1. Pull the Updated Service
    ```bash
    $ cd ~/mbot_ws
    $ git clone https://github.com/mbot-project/mbot_sys_utils.git
    $ cd ~/mbot_ws/mbot_sys_utils/services
    $ ./install_mbot_services.sh
    ```
2. Restart the OLED service to apply the changes. You’ll need to enter your mbot password.
    ```bash
    $ mbot service restart mbot-oled.service
    ```
    At this point, the second page on your OLED screen should show as "Not Available".

    <a class="image-link" href="/assets/images/botlab/new-feature/oled-battery1.jpg">
    <img src="/assets/images/botlab/new-feature/oled-battery1.jpg" alt=" " style="max-width:400px;"/>
    </a>
3. Pull Updates from `mbot_lcm_base`
    ```bash
    $ cd ~/mbot_ws/mbot_lcm_base
    $ git pull
    $ ./scripts/install.sh
    ```
4. Pull Upstream Updates from `mbot_firmware`, re-compile the firmware and flash the firmware (not calibration) same as the system setup guide you did:
    ```bash
    $ cd ~/mbot_ws/mbot_firmware
    $ cd build
    $ cmake -DMBOT_TYPE=CLASSIC -DENC=48 ..
    $ make
    # then put the controlboard to Boot Mode
    $ sudo mbot-upload-firmware flash <mbot_calibrate_classic_<vX.X.X>_enc48.uf2>
    ```
5. Restart the OLED Service Again
    ```bash
    $ mbot service restart mbot-oled.service
    ```
    Now the second page should display the battery voltage.

    <a class="image-link" href="/assets/images/botlab/new-feature/oled-battery2.jpg">
    <img src="/assets/images/botlab/new-feature/oled-battery2.jpg" alt=" " style="max-width:400px;"/>
    </a>

    If the voltage drops below 9V, the screen will flash a low-battery warning. (Note: The wavy effect you might see in videos is due to the camera’s frame rate—it’s not visible to the human eye, so in real life, the flashing looks smooth.)
    <iframe width="560" height="315" src="https://www.youtube.com/embed/clWTYvQ57FA?si=ubit_6U4WYcHmZZ6" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

**Troubleshooting**

If everything went smoothly but the voltage reads as -1, it means the battery voltage isn't updating. This could indicate that your LCM serial server isn't working. Try these steps:
- Run minicom to check if data is being printed. Should look like this:
    ```bash
    |  COMMS OK  TIME: 1731015352395904 |
    |-----------------------------------------------|
    | ANALOG                                        |
    |  AIN 0    |  AIN 1    |  AIN 2    |  BATT (V) |
    |-----------|-----------|-----------|-----------|
    |    0.3069 |    0.3208 |    0.3083 |   11.0046 |
    |-----------------------------------|
    | ENCODERS                          |
    |  ENC 0    |  ENC 1    |  ENC 2    |
    |-----------|-----------|-----------|
    |         1 |         0 |         0 |
    |-----------------------------------|
    | IMU                               |
    |  ROLL     |  PITCH    |  YAW      |
    |-----------|-----------|-----------|
    |   -0.0502 |    0.0763 |    0.0008 |
    |-----------------------------------|
    | MOTOR                             |
    |  MOT 0    |  MOT 1    |  MOT 2    |
    |-----------|-----------|-----------|
    |    0.0000 |    0.0000 |    0.0000 |
    |-----------------------------------|
    | ODOMETRY                          |
    |  X        |  Y        |  THETA    |
    |-----------|-----------|-----------|
    |    0.0000 |    0.0000 |    0.0000 |
    |-----------------------------------|
    ```
- If no data appears, press the `RST` button on your control board and see if it resumes.
- If data does appear, your did not flash the latest firmware successfully.
