---
layout: default
title: Fetch updates from upsteam repo
nav_order: 3
grand_parent: Armlab
parent: How-to Guide
last_modified_at: 2023-12-13 18:37:48 -0500
---

There may be changes made during the semester. To keep your codebase in sync with the upstream code, you will sometimes need to fetch updates from the original repository. Here's how to do it:
1. First, commit and push your local changes to synchronize them with the cloud. Then, click the `Update fork` button on the webpage of your team's repository. GitLab will automatically fetch and merge everything for you on the remote side. 
    <a class="image-link" href="/assets/images/armlab/setup-guide/update-fork.png">
    <img src="/assets/images/armlab/setup-guide/update-fork.png" alt="" style="max-width:600px;"/>
    </a>
    - Alternatively, if you are comfortable with git commands, you can add a remote branch, then fetch and merge locally.
2. Next, pull the updated team code locally:
    1. Run “git fetch”  - to fetch the latest updates
    2. Run “git status” - git will tell you that your branch is behind
    3. Run “git pull” - this time git will pull the code for you, and the changes will be applied locally


