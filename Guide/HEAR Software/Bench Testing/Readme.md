# Overview

This document describes how to test the software stack in the actual hardware and run the PX4 in the drone.

# Pre-requisites
SITL testing has been followed

## Differentiators from SITL Testing

### Mavlink Router Settings

At 

```bash
mavlink-routerd 0.0.0.0:14540
```

### Make sure you disconnect ORIN and RPI from the switch
To prevent multiple LeafFC instances from connecting to PX4

### Choose the Bench-Testing profile instead of the SITL profile from the client web app