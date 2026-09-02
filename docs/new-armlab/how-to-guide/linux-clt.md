---
layout: default
title: Linux Command Line Tools
nav_order: 1
grand_parent: New Armlab
parent: How-to Guide
last_modified_at: 2026-09-02 12:00:00 -0400
---

> Almost everything in this lab happens in a terminal. This page covers the commands you will actually use, and a few habits that will save you a lot of time. You do not need to memorize it — skim it once, then come back when you get stuck.

### Contents
* TOC
{:toc}

## Terminal basics

{: .note}
Open a terminal: **ctrl+alt+t** <br>
Copy/paste in a terminal: **ctrl+shift+c** / **ctrl+shift+v** (the extra shift matters because plain ctrl+c means something else, it sends a signal interupt to stop a running process)

Four habits worth building immediately:

| Key | What it does |
| --- | ------------ |
| **Tab** | Auto-completes the file, folder or command you are typing. Press it constantly |
| **Up / Down arrow** | Scrolls through commands you already ran, so you can rerun or edit one |
| **ctrl+c** | Stops the program that is currently running. This is your escape hatch |
| **ctrl+l** | Clears the screen (so does typing `clear`) |

Tab completion is not just a convenience but also a check. If a name does not complete, you have a typo, or you are not in the folder you think you are in.

{: .note}
If Tab does not complete a script name, the script may not be executable. See [Running scripts and programs](#running-scripts-and-programs).

### Reading the prompt

```
(env550lab) student@ROB550-AM:~/armlab$
```

- `(env550lab)` — the active conda environment. If you do not see this, your code will not run
- `student` — the user you are logged in as
- `ROB550-AM` — the machine name
- `~/armlab` — **where you are right now**. `~` means your home directory
- `$` — end of the prompt; what you type goes after it

## Finding your way around

### `pwd` — where am I?

```bash
pwd
```

```
/home/student/armlab
```

### `ls` — what is here?

```bash
ls
```

```
install_scripts  README.md  src
```

`ls` takes flags that make it much more useful:

| Command | What it shows |
| ------- | ------------- |
| `ls` | Names only |
| `ls -l` | Long form: permissions, owner, size, modification date |
| `ls -a` | Includes hidden files (anything starting with `.`, like `.git`) |
| `ls -lh` | Long form with human-readable sizes (`4.0K`, `1.2M`) |
| `ls -lt` | Sorted by time, newest first — handy for "which file did I just create?" |

Flags combine, so `ls -alh` is all three at once.

### `cd` — move somewhere else

```bash
cd src
```

Useful destinations:

```bash
cd ~          # home directory
cd ..         # up one level, to the parent directory
cd -          # back to the directory you were in before
cd            # with no argument, also goes home
```

- `.` means "the directory I am in right now"
- `..` means "one directory up"

### Absolute vs. relative paths

An **absolute** path starts with `/` and works from anywhere:

```bash
cd /home/student/armlab/src
```

A **relative** path starts from where you currently are:

```bash
cd src/                # into src, from armlab
cd ../install_scripts  # up one level, then into install_scripts
```

{: .important}
Your armlab repo **must** be cloned directly into your home directory (`~/armlab`). Several scripts assume that path, and the control station will not run if you nest it inside another folder.

## Looking at files

| Command | Use it for |
| ------- | ---------- |
| `cat file.py` | Dump a short file to the screen |
| `less file.py` | Page through a long file. Arrows/PgUp/PgDn to move, `/word` to search, **`q` to quit** |
| `head -n 20 file.py` | First 20 lines |
| `tail -n 20 file.py` | Last 20 lines |
| `tail -f log.txt` | Last lines, then **keep watching** as the file grows — good for logs while a program runs |

{: .note}
If you find yourself stuck in a full-screen program with no prompt, you are probably in `less` or `man`. Press `q`.

## Creating, copying, moving and deleting

```bash
mkdir new_folder            # create a folder
mkdir -p a/b/c              # create nested folders, no complaints if they exist
touch notes.txt             # create an empty file
cp file.py backup.py        # copy a file
cp -r folder/ backup/       # copy a folder and everything in it
mv old.py new.py            # rename a file
mv file.py src/             # move a file into a folder
rm file.py                  # delete a file
rm -r folder/               # delete a folder and its contents
```

`mv` is both "move" and "rename" — renaming is just moving to a new name in the same place.

The `-r` flag on `cp` and `rm` stands for **recursive**: apply this to the folder and everything inside it.

{: .warning}
`rm` is permanent. There is no trash can and no undo. Before running `rm -r`, run `ls` on the same path and read what is about to disappear. Be especially careful combining `rm -r` with `*`.

## Wildcards

`*` matches any number of characters, which lets one command act on many files:

```bash
ls *.py                       # every Python file here
rm calib_images/*.png         # every PNG in that folder
chmod +x install_scripts/*.sh # every shell script in that folder
```

{: .warning}
Wildcards expand *before* the command runs. `rm *` deletes everything in the current directory. Get in the habit of running `ls` with the same pattern first to see what it matches.

## Finding things

### `grep` — search inside files

This is one of the most valuable commands you will learn. It finds text *inside* files, which is how you locate a function or a variable in a codebase you did not write.

```bash
grep -rn "set_position" src/
```

```
src/lite6arm.py:142:    def set_position(self, x, y, z):
src/state_machine.py:88:        self.rxarm.set_position(...)
```

| Flag | Meaning |
| ---- | ------- |
| `-r` | Search recursively through a folder |
| `-n` | Show line numbers |
| `-i` | Ignore case |
| `-w` | Match whole words only |

### `find` — search for files by name

```bash
find . -name "*.png"        # every PNG below the current directory
find ~ -name "camera.py"    # locate a file somewhere in your home directory
```

## Running scripts and programs

To run a script in the current directory, you must put `./` in front of it. The `./` says "the one right here":

```bash
./install_scripts/install_anaconda.sh
```

If you get `Permission denied`, the file is not marked executable. Fix it with `chmod +x` ("change mode, add execute"):

```bash
chmod +x install_scripts/*.sh
```

{: .note}
This is exactly the fix referenced in the [Setup Guide](/docs/new-armlab/setup-guide#installation). A script that is not executable also will not Tab-complete, which is often the first clue.

To run a Python program:

```bash
cd src
python control_station.py
```

`which` tells you where a command actually comes from — useful for confirming you are running the Python inside your conda environment and not the system one:

```bash
which python
```

```
/home/student/anaconda3/envs/env550lab/bin/python
```

## Permissions, briefly

`ls -l` shows permissions in the first column:

```
-rwxr-xr-x  1 student student  2.1K Sep  2 10:14 install_camera.sh
-rw-r--r--  1 student student  8.4K Sep  2 10:14 camera.py
```

Read the first ten characters as: type, then three groups of **r**ead/**w**rite/e**x**ecute for owner, group and everyone else. The `x` on the first file is what makes it runnable; the second file has no `x` anywhere, so `./camera.py` would fail.

## Managing running programs

| Action | How |
| ------ | --- |
| Stop the running program | **ctrl+c** |
| See what is running | `ps aux \| grep python` |
| Force-kill a process by PID | `kill -9 <PID>` |
| Interactive process viewer | `htop` (`q` to quit) |
| Suspend the running program | **ctrl+z** |
| Resume it in the background | `bg` |
| Bring it back to the foreground | `fg` |

{: .important}
A stale Python process will hold onto the camera or the arm connection and make the next run fail with a "device busy" error. When something refuses to start and you cannot see why, check `ps aux | grep python` for a leftover process and kill it.

A program that occupies the terminal is not stuck — that is normal. Open a second terminal (**ctrl+alt+t**) to run something else at the same time.

## Pipes and redirection

The `|` character sends the output of one command into another:

```bash
ps aux | grep python        # list processes, keep only the Python ones
ls -l | less                # page through a long listing
```

Redirection sends output to a file instead of the screen:

```bash
python control_station.py > out.txt        # send normal output to a file
python control_station.py > out.txt 2>&1   # send errors as well
python control_station.py 2>&1 | tee out.txt   # save to a file AND still watch it
```

{: .note}
`2>&1` means "send error messages to the same place as normal output". When you are asked to post an error to the course Discord, this is how you capture the whole thing rather than retyping it.

## Installing software

```bash
sudo apt update              # refresh the list of available packages
sudo apt install git         # install a package
```

`sudo` runs a command as the administrator and will ask for a password.

{: .warning}
Only use `sudo` when a command genuinely needs it, such as `apt`. Running your lab code or conda commands with `sudo` creates root-owned files in your home directory that you will then have trouble editing.

## Checking hardware and the network

These matter in this lab specifically — the camera is on USB and the arm is on the network.

```bash
lsusb                        # list USB devices; confirm the camera is attached
rs-enumerate-devices -s      # RealSense-specific: model, serial, USB version
ip addr                      # this laptop's network interfaces and IP addresses
ping 192.168.1.xxx           # can this laptop reach an address? ctrl+c to stop
```

{: .note}
If the control station cannot connect to the arm, `ping` the address you set as `XARM_IP` before you start debugging your code. If `ping` does not get replies, the problem is the cable, the IP, or the arm — not your Python.


## Conda in the terminal

Your terminal should start in the `env550lab` environment automatically. If the prompt is missing `(env550lab)`:

```bash
conda env list               # show environments; * marks the active one
conda activate env550lab     # activate it by hand
```

{: .warning}
If `import pyrealsense2` or `import xarm` fails, check the prompt before anything else. Running with the system Python instead of `env550lab` produces exactly this error.

## Getting help

```bash
ls --help          # quick summary of a command's options
man ls             # the full manual page (q to quit)
```

## Quick reference

| Command | Does |
| ------- | ---- |
| `pwd` | Print current directory |
| `ls -lh` | List files with details |
| `cd <dir>` | Change directory |
| `cd ..` | Up one level |
| `mkdir -p <dir>` | Create folder(s) |
| `cp -r <src> <dst>` | Copy |
| `mv <src> <dst>` | Move or rename |
| `rm -r <dir>` | Delete (permanently) |
| `cat` / `less` | View a file |
| `tail -f <file>` | Watch a file grow |
| `grep -rn "text" <dir>` | Search inside files |
| `find . -name "<pat>"` | Search for files |
| `chmod +x <file>` | Make a script executable |
| `./<script>` | Run a script here |
| `which python` | Find which program will run |
| `ps aux \| grep python` | Find running processes |
| `kill -9 <PID>` | Force-kill a process |
| `sudo apt install <pkg>` | Install software |
| `ping <ip>` | Test network reachability |
| `conda activate env550lab` | Activate the lab environment |
| `man <cmd>` | Read the manual |
