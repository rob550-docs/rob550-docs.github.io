# ROB550 BotLab

> This repository contains guides for the Mbot Classic for ROB550.

## How to modify the files here:

### 1. Set up environment and host locally
1. Install Jekyll: install all the prerequisites following the instruction there: [Jekyll doc](https://jekyllrb.com/docs/)
2. Since we already have this site, after install jekyll and bundler gems by running `gem install jekyll bundler`, we can directly cd to this folder, then run
    ```
    bundle exec jekyll serve
    ```

Now we are hosting it locally and can access to the website from http://localhost:4000.

### 2. Modify the existed articles
All the articles are located under `\docs`.
```
.
├── staff-guide
│   ├── index.md
│   └── jetson-image.md
├── student-guide
│   ├── index.md
│   ├── mbot-classic-assemly.md
│   ├── mbot-how-to-guide.md
│   └── mbot-system-setup.md
└── troubleshooing.md
```
- Each level of the structure needs an `index.md` to indicate where they are at in the hierarchy.

```
---
layout: default
title: Student Guide
nav_order: 2
has_children: true
last_modified_at: 2023-09-25 14:37:48 -0500
---
```
- Within each `.md` file, there is this header to config each article.  

After modifying the articles, simply push them to the GitHub repository. The GitHub Action will then build and deploy everything.

> Most of the content is in markdown, but there are some of HTML syntax there for images pop up feature because of using plugin `magnific-popup`. I couldn't figure how to intergrate it into markdown syntax :c