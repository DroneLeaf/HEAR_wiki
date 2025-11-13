# Overview

This document describes how to test the software stack in the actual hardware and run the PX4 in the drone.

# Pre-requisites
SITL testing has been followed

Refer to `Guide/External Software/MAVLink/mavlink_arch.drawio.svg` for network setup

## Differentiators from SITL Testing

### Mavlink Router Settings

Make sure any systemd MAVLink router has been stopped first:

```bash
sudo systemctl stop mavlink-router.service
```

Then run 

```bash
mavlink-routerd 0.0.0.0:14540 0.0.0.0:14550
```

### Make sure you disconnect ORIN and RPI from ethernet switch
To prevent multiple LeafFC instances from connecting to PX4

### Choose the Bench-Testing profile instead of the SITL profile from the client web app