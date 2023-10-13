---
layout: default
title: Doxygen
parent: Staff Guide
nav_order: 2
last_modified_at: 2023-10-13 14:20:48 -0500
---

> The guide introduces how the code library was generated. The content in this doc is only true for Ubuntu users

## Install Doxygen

- If you want to build from source 
    - Go to the [official Doxygen website](https://www.doxygen.nl/download.html) and download the source distribution.
    - Following the [Installation Instruction](https://www.doxygen.nl/manual/install.html), there are tools need to be installed before use Doxygen. Here are some of the commands you can use directly to save time.
        ```bash
        # install GNU Tools - flex, bison, libiconv, and GNU make
        $ sudo apt-get install flex bison libiconv-hook-dev make
        # install Cmake, LaTex, visualization toolkit (graphviz), ghostscript
        $ sudo apt-get install cmake texlive-full graphviz ghostscript
        # install Qt5 stuff
        $ sudo apt-get install qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools  
        $ sudo apt install qtcreator
        # use qmake -v to verify Qt installation
        ```

- Or you can do this
```bash
# Check the package info
$ apt show doxygen
$ sudo apt-get install doxygen
```
    -  If you want to have an additional functionality such as generating graphs, you would need to install Graphviz separately.

## Config Doxygen
1. Create workspace and then create Doxyfile
```bash
$ mkdir botlab_ws
$ doxygen -g config_file_name
```
    - If you omit the file name, a file named Doxyfile will be created.

3. Edit the Config file

    These are the few lines I changed in the default Doxyfile.
    ```
    PROJECT_NAME           = "MBot Software Library"
    OUTPUT_DIRECTORY       = doxygen_ws
    RECURSIVE              = YES
    EXTRACT_ALL            = YES
    GENERATE_LATEX         = NO
    ```

## Generate the doc  

```bash
$ cd ~/botlab_ws
$ mkdir doxygen_ws
# generate the docs
$ doxygen Doxyfile
```

Created a doxygen workspace where the doxygen will output the document to. And the `/html` is all you need. We can read the document locally by access the `/html/index.html` page. Below is my file system for example.
```
botlab_ws
├── Doxyfile
├── doxygen_ws
│   └── html
├── mbot_autonomy
└── mbot_firmware
```

## Host it on GitHub Pages
Rename html folder to be docs because that's our savior GitHub Pages wants
```bash
$ cd botlab_ws/doxygen_ws
$ mv html docs
```

Clone the `doxygen_docs` repository, replace your `docs` folder with the old one and push, the GitHub Pages will be automatically updated.