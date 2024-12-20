# MAVLINK Router Installation and Configuration Guide
This guide will help you to install and configure the MAVLink Router on your system (FC).

## Prerequisites

- Python 3.8 or later
- CMake 3.10 or later
- GCC 9 or later
- Git
- mason
- ninja

## Install the required packages

You can install the required packages using the following command:

```bash
sudo apt-get install python3 python3-pip
sudo pip3 install meson
sudo apt-get install build-essential ninja-build
```

## Build and Install MAVLink Router

Clone the mavlink-router repository from the GitHub repository using the following command:

```bash
git clone git@github.com:DroneLeaf/mavlink-router.git
cd mavlink-router
git submodule update --init --recursive
meson setup build .
ninja -C build
sudo ninja -C build install
```

## Usage

Consider running your flight controller (FC) and the MAVLink Router on the same system. You can run the MAVLink Router using the following command:

```bash
mavlink-routerd -e <KNOWN_CLIENT_ENDPOINT_ADDR>  <PX4_MAVLINK_REMOTE_ADDR>
mavlink-routerd -e 127.0.0.1:14650  0.0.0.0:14550 # PX4 SITL example
mavlink-routerd -e 127.0.0.1:14650  192.168.144.4:14550 # PX4 RPI example
```

Then you can connect QGC to the FC device's IP address and port 5770 using **TCP** protocol.