---
layout: default
title: Doxygen
parent: Staff Guide
nav_order: 2
last_modified_at: 2023-10-23 15:20:48 -0500
---

> The guide introduces how the code library was generated. The content in this doc is only true for Ubuntu users

## Install Doxygen
```bash
# Check the package info
$ apt show doxygen
$ sudo apt-get install doxygen
```

## Clone the doxygen repo
Clone the doxygen repo to your local mbot workspace.
The Doxyfile is designed to use in this file structure:
```
.
├── doxygen_docs
│   ├── docs
│   ├── Doxyfile
│   ├── main_page.md
│   └── mbot-logo.png
├── mbot_firmware
└── example_project1
```

## Generate the doc  

```bash
$ cd doxygen_docs/
# generate the docs
$ doxygen Doxyfile
```

After you generate the new documents, all the updates you have made in either the `Doxyfile` or `projects` will be output to the /docs folder.

## Preview
You can use the `Live Server` VS Code extension to view the generated index.html file.

## Host on Github
After making the updates, a git push will automatically trigger the GitHub Action, which will deploy the new documentation website for you.