---
layout: default
title: Misc
parent: How-to Guide
grand_parent: Botlab
nav_order: 5
last_modified_at: 2023-10-19 13:37:48 -0500
---

> This guide provides instructions on miscellaneous how-to questions.

### Contents
* TOC
{:toc}

## How to Turn on and off the robot

Assuming you have finished the headless setup
- To turn on, just switch on the power bank to `I` position
- To turn off
    - NoMachine GUI: use the Ubuntu system poweroff
    - Command line tool over SSH connection: `sudo shutdown -h now`

 
## How to charge the power bank
1. Push the switch of the power bank to position "I" to make the charging circuit breakout. 
2. Connect the charger to the power bank
3. The LED indicator on the AC-DC charger head showing RED means the charging process is working. 
4. The LED indicator on the AC-DC charger head showing GREEN means the charging process has completed. 

## How to transfer file from MBot to your laptop - wormhole

There is a command-line tool, called [wormhole](https://magic-wormhole.readthedocs.io/en/latest/welcome.html) that comes in handy. It can safely and conveniently transfer things from one computer to another.

The scenario with the wormhole tool is as follows: If you've recorded a log file on the Jetson, or just have some files need to transfer it to your laptop without uploading it to Github, you can use wormhole for this purpose. Below is an example of its usage:

<a class="image-link" href="/assets/images/how-to/wormhole-tool.png">
<img src="/assets/images/how-to/wormhole-tool.png" alt=" " style="max-width:600px;"/>
</a>

```bash
# On both Jetson and your laptop, install the tool
$ sudo apt install magic-wormhole
# then on Jetson, send the file:
$ wormhole send file_you_want_to_send
# then on your laptop, receive the file:
$ wormhole receive code-wormhole-generated
```
