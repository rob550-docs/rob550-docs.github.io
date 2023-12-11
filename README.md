# ROB550 BotLab

> This repository contains guides for the Mbot Classic for ROB550.

## How to modify the files on github:
Simply using the edit feature on GitHub is sufficient. Once you save, the GitHub Action will automatically build and deploy everything. This is the best method for quick typo fixes. If you wish to modify a large portion of an article or write a new one, you may need to follow these steps to edit and preview it locally.

## How to modify the 550 docs site locally:

### 1. Set up environment and host locally
1. Install Jekyll: install all the prerequisites following the instruction there: [Jekyll doc](https://jekyllrb.com/docs/)
2. Since we already have the site, after install jekyll and bundler gems by running `sudo gem install jekyll bundler`, we can directly cd to this folder, then run
    ```
    bundle exec jekyll serve
    ```
    - You might encounter error for the first time set up, you need to run `sudo bundle install` as the error message indicates, then run the command above.

Now we are hosting it locally and can access to the website from http://localhost:4000.

### 2. Modify the existed articles
All the articles are located under `/docs` and images are under `/assets/images`
```
.
├── armlab
│   ├── checkpoints
│   │   ├── checkpoint1.md
│   │   ├── ...
│   │   └── index.md
│   ├── hardware.md
│   ├── how-to-guide
│   │   ├── index.md
│   │   └── linux-clt.md
│   ├── index.md
│   ├── setup-guide.md
│   └── software.md
├── botlab
│   ├── ...
├── how-to-guide
│   ├── ...
└── staff-guide
    ├── ...

```
- Each level of the structure needs an `index.md` to indicate where they are at in the hierarchy. Other files are the actual site content.

Take the lowest level post `Checkpoint 1` in armlab for example.

```
---
layout: default
title: Checkpoint 1
nav_order: 1
parent: Checkpoints
grand_parent: Armlab
last_modified_at: 2023-11-30 14:37:48 -0500
---
```
- Within each `.md` file, there is this header to config each article. Here we can tell that "Checkpoint 1" is under the parent folder "Checkpoints", and "Checkpoints" is under "Armlab" which is the highest level in our structure.
- More detail at the [Just the Docs](https://github.com/just-the-docs/just-the-docs) official website.

After modifying the articles, simply push them to the GitHub repository. The GitHub Action will then build and deploy everything.

> Most of the content is in markdown, but there are some of HTML syntax there for images pop up feature because of using plugin `magnific-popup`. I couldn't figure how to intergrate it into markdown syntax :c