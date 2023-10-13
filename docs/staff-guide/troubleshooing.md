---
layout: default
title: Troubleshooting
parent: Staff Guide
nav_order: 3
last_modified_at: 2023-10-13 17:37:48 -0500
---

### Services are failed
Use journalctl to see error logs of the service can usually give a better idea of what's going wrong.
```bash
$ sudo journalctl -u your_failed_service.service
```
If the `mbot-publish-info.service` is failed, you can go to `/var/log/mbot/mbot_pub_info.log` to see the detailed log.