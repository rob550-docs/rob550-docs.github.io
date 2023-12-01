---
layout: default
title: Linux Command Line Tools
nav_order: 1
grand_parent: Armlab
parent: How-to Guide
last_modified_at: 2023-11-30 18:37:48 -0500
---

> This post introduces you to the most basic command line tools you need to know for the class.

{: .note}
How to open a terminal: **ctrl+alt+t** <br>
How to copy/paste in a terminal: **ctrl+shift+c/ctrl+shift+v**

Copy the commands below and paste into the terminal, then hit ENTER to see similar result:

- How to check where we are at in current terminal:
    ```bash
    $ pwd
    ```
    For example:
    ```
    xssun@ROB-STAFF-10U:~$ pwd
    /home/xssun
    ```
    - Here `/home/xssun` is current directory we are at

- How to create a new folder and confirm the creation:
    ```bash
    # create a new folder
    $ mkdir new_folder
    # list directory contents
    $ ls
    ```
    For example:
    ```
    xssun@ROB-STAFF-10U:~$ mkdir parent_folder
    xssun@ROB-STAFF-10U:~$ ls
    Desktop    Downloads  parent_folder  Public  Documents  
    Music      Pictures   Videos 
    ```

- How to navigate to certain folder:  
    ```bash
    $ cd destination_folder
    ```
    For example:
    ```
    xssun@ROB-STAFF-10U:~$ cd parent_folder/
    xssun@ROB-STAFF-10U:~/parent_folder$ pwd
    /home/xssun/parent_folder
    ```

- More about nagivation:
    ```bash
    # move to home directory
    $ cd ~
    # move to parent directory
    $ cd ..
    ```
    - In filesystems, we use the double dot (..) to access the parent directory, whereas the single dot (.) represents the current directory.

- How to remove a file/folder:
    ```bash
    $ rm the_file
    $ rm -r the_folder
    ```
    - The `-r` option stands for recursive. When used with `rm`, it recursively deletes all the files and subdirectories within the specified directory, starting from the deepest level and moving up.
    
    For example:
    ```
    xssun@ROB-STAFF-10U:~$ rm -r parent_folder/
    xssun@ROB-STAFF-10U:~$ ls  
    Desktop    Downloads  Public  Documents  Music      
    Pictures   Videos
    ```