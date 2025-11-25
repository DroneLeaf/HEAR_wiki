<!-- this page is taken from source. Clearly label this in a correct MD file format -->
---
title: QGroundControl Running Dependencies
slug: Guide/External Software/QGroundControl/QGC_running_dependencies
source: `https://raw.githubusercontent.com/mavlink/qgroundcontrol/refs/tags/v4.4.2/docs/en/qgc-user-guide/getting_started/download_and_install.md`
tags: 
  - Guide
  - External Software
  - QGroundControl
  - Dependencies
---
---
## QGroundControl Running Dependencies  
# Download and Install

Download and install the LeafMC (QGC) from software-stack releases.

Then:

## Ubuntu Linux {#ubuntu}

_QGroundControl_ can be installed/run on Ubuntu LTS 20.04 (and later).

Ubuntu comes with a serial modem manager that interferes with any robotics related use of a serial port (or USB serial).
Before installing _QGroundControl_ you should remove the modem manager and grant yourself permissions to access the serial port.
You also need to install _GStreamer_ in order to support video streaming.

Before installing _QGroundControl_ for the first time:

1. On the command prompt enter:
   ```sh
   sudo usermod -a -G dialout $USER
   sudo apt-get remove modemmanager -y
   sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
   sudo apt install libqt5gui5 -y
   sudo apt install libfuse2 -y
   ```
   <!-- Note, remove install of libqt5gui5 https://github.com/mavlink/qgroundcontrol/issues/10176 fixed -->
1. Logout and login again to enable the change to user permissions.

&nbsp;
To install _QGroundControl_:

1. Download [QGroundControl.AppImage]
1. Install (and run) using the terminal commands:
   ```sh
   chmod +x ./QGroundControl.AppImage
   ./QGroundControl.AppImage  (or double click)
   ```

::: info
There are known [video steaming issues](../troubleshooting/qgc_setup.md#dual_vga) on Ubuntu 18.04 systems with dual adaptors.
:::

::: info
Prebuilt _QGroundControl_ versions from 4.0 cannot run on Ubuntu 16.04.
To run these versions on Ubuntu 16.04 you can [build QGroundControl from source without video libraries](https://dev.qgroundcontrol.com/en/getting_started/).
:::
