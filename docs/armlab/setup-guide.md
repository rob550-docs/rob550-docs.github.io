---
layout: default
title: Setup Guide
nav_order: 1
parent: Armlab
last_modified_at: 2023-11-30 14:37:48 -0500
---

> This guide will walk you through the setup steps you need before starting Armlab.

### Contents
* TOC
{:toc}
 
## Login to the laptop
In Armlab, each station is provided with a laptop running Ubuntu 22.04:
- If you are in the morning session, log in to **student_am**
- If you are in the afternoon session, log in to **student_pm**

Same password for both users: **i<3robots!**

## Fork codebase
### 1. Sign into Umich GitLab 

<a class="image-link" href="/assets/images/armlab/setup-guide/gitlab_login.png">
<img src="/assets/images/armlab/setup-guide/gitlab_login.png" alt="" style="max-width:300px;"/>
</a>

The code that we will be using for this lab is hosted on GitLab. Go to this [link](https://gitlab.eecs.umich.edu/) and use your UM credentials to sign into your account.

### 2. Create a Group on GitLab and Adding Members
One group member needs to create a GitLab Group and add all members. Follow these steps:
1. On the left-hand menu, click on "Groups" to access the [Groups page](https://gitlab.eecs.umich.edu/dashboard/groups), then click the "New Group" button to initiate the group creation process.
2. Group Details:
    - Group name: enter `armlab-s<SECTION#>_g<GROUP #>` (i.e. armlab-s012_g7)
    - Visibility level: select “Private”
    - Leave other fields blank
3. Invite your team members as “owners” of the project
    - Firstly, go to groups page: sidebar -> groups
    - Click your team group
    - Add members: sidebar -> Manage -> Members -> invite members


### 3. Fork Repositories
"Forking a repository" refers to the process of creating a personal copy of a repository (a collection of files and code) hosted on a platform like GitHub or GitLab, in our case is GitLab. This copy is entirely separate from the original repository, allowing you to make changes and updates without affecting the original project. In order to create a fork, complete the following steps:

1. Navigate to the repository you’d like to fork, in this case, you should fork the [armlab’s repository](https://gitlab.eecs.umich.edu/rob550-f23/armlab-f23)
2. On the top right, select "Fork"
3. You can change the name of the repository if you would like, but the key thing is to change the "Project URL" field so that it is forked into the group you just created.
4. Change the visibility level to "Private".

    <a class="image-link" href="/assets/images/armlab/setup-guide/fork_project.png">
    <img src="/assets/images/armlab/setup-guide/fork_project.png" alt="" style="max-width:500px;"/>
    </a>
    
### 4. Clone to local
1. Go to your group’s armlab repository webpage and copy the URL of “Clone with HTTPS”

    <a class="image-link" href="/assets/images/armlab/setup-guide/clone.png">
    <img src="/assets/images/armlab/setup-guide/clone.png" alt="" style="max-width:400px;"/>
    </a>

2. Then open a Terminal (shortcut: ctrl+alt+t), run the following commands: 
```bash
# move to home directory
$ cd ~
# clone the codebase to your laptop
$ git clone https://your_groups_url 
```
- To copy-paste anything in the terminal, we use `ctrl+shift+c`/`ctrl+shift+v`
- This command will prompt you for a username and password. Please enter your UMICH credentials

3. After cloning the code, enter this command in the terminal to see the cloned directory:
```bash
# ls will show all the files/folders under the current directory
$ ls
```

    {: .note}
    The commands we use here are known as **Linux command line tools**. Understanding the basics of Linux CLT is essential for smooth progress in this class. If you are not familiar with it, [this post](/docs/armlab/how-to-guide/linux-clt) provides some basic information you need to know. 

{: .sanity_check}
At this point, you should have your team forked Armlab codebase downloaded/cloned on the Ubuntu laptop at your team's station.

## Installation
To use the code, firstly, we need to install all the dependencies/SDKs we need. 
### 1. Install all the dependencies and packages
    
If this is your first time setting up, you need to install the necessary dependencies and packages. Open a terminal and navigate to folder `/install_scripts`. Then, run the following command:
```bash
$ ./install_Dependencies.sh
```
- Wait until it's complete before proceeding to the next step.

### 2. Install interbotix arm 

Run the following command:
```bash
$ ./install_Interbotix.sh
```
During the installation, you'll encounter prompts. For prompts related to AprilTag and MATLAB-ROS installation, type **no** and press Enter.

<a class="image-link" href="/assets/images/armlab/setup-guide/interbotix_install.png">
<img src="/assets/images/armlab/setup-guide/interbotix_install.png" alt="" style="max-width:600px;"/>
</a>

{: .warning}
Be cautious with the prompts and enter **exactly** what is specified in the image. We install the perception module separately. If you enter 'Yes' to the perceptoin module, it will result in the installation of a duplicate AprilTag module, leading to configuration issues.

Wait until it's complete before proceeding to the next step.

### 3. Move config files

Run the following command:
```bash
$ ./install_LaunchFiles.sh
```
- This file is used to move the config files. The configurations are based on the AprilTag family we have and the specific camera model we use.

### 4. Install camera calibration package

**Open a new terminal**, then run the following command:
```bash
$ ./install_Calibration.sh
```

### 5. Set up ROS_DOMAIN_ID

Run the following command:
```bash
$ echo "export ROS_DOMAIN_ID=your_station_number" >> ~/.bashrc
```
- Note, replace `your_station_number` with the actual number on the wall next to your station's desk
- To check if you set it successfully, open a new terminal and run:
```bash
$ printenv | grep ROS
```
    - where you should see the output has all the environmental variables related to ROS, and you should see ROS_DOMAIN_ID=the_number_you_set print out in the terminal


### 6. Reboot
If you have successfully completed all the steps above, please reboot the computer.
```bash
$ sudo reboot
```

## Testing
After successfully installing all the necessary components for the workstation, now it is time to test.

By launching all the ROS nodes we need, you are starting the camera, the apriltag package, and the arm, along with a control station GUI where you can manually control the arm. We will talk about how to control the arm later in the checkpoint 1. 

If you launch everything successfully, this will be the scene on your laptop:

- To quit control station, click the “exit” button on the top right corner
- To quit other nodes, hit ctrl+c in the terminal

{: .warning}
Always put the arm in the sleep position when you stop using the arm node! <br>
Otherwise the arm loses torque will then splash on the board  : D



## Fetch from upstream