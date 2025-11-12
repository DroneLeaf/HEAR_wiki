# DroneLeaf Software Stack

Welcome to DroneLeaf software stack!

This documentation is targeted for developers who wanted to contribute to the DroneLeaf software stack.

It serves as the master documentation and entry point for all things DroneLeaf.

## Anatomy of the software stack

<!-- https://chatgpt.com/share/69142062-1618-8002-8b69-7ab6486b046c -->
<!-- embed ./Media/anatomy.mmd-->

Currently, there are two stages of operation: 
1. **Commissioning**: mainly a process on the web, coordinated with the edge device.
2. **Flight**: the process of flying the drone.

And there are two types of environments:
1. **Deployment**: This is where actual flight happens.
2. **Development**: Happens in two possible ways.
    - **Simulation in the loop (SITL)**: Simulates a real drone visually allowing for safe and convenient testing.
    - **Bench**: Adds a physical pixhawk hardware.

The Deployment setup anatomy is given by the following diagram:
<img src="../Media/anatomy_deployment.svg" alt="DroneLeaf Software Stack Anatomy">

The Development SITL setup anatomy is given by the following diagram:
<img src="../Media/anatomy_development_sitl.svg" alt="DroneLeaf Software Stack Anatomy">

And finally, the Development Bench setup anatomy is given by the following diagram:
<img src="../Media/anatomy_development_bench.svg" alt="DroneLeaf Software Stack Anatomy">

# Getting started with Deployment (Targeted for DroneLeaf clients)
TODO: refer to Knowledge Base

# Getting Started with Development 
Development in the SITL environment involves working with four stacks:

- **Flight stack:** represented by the software-stack repo. TODO link
- **Petals stack:** a python code base for added value functionality.
- **Controller dashboard:** 
- **Web client application (fly.droneleaf.io):**

## Preparing the Developer Machine
### Prerequisites
- Ubuntu 20.04 LTS
- Hardware: minimum 16GB RAM, 4-core CPU (x86_64/AMD64), 256GB free disk space

### OS Installation
- Fresh installation [recommended] TODO
For full details, see the development-machine-OS-installation guide: [Guide/Hardware and Process/Development Machine Preparation/development-machine-OS-installation-for-droneleaf-stack.md](./../Hardware%20and%20Process/Development%20Machine%20Preparation/development-machine-OS-installation-for-droneleaf-stack.md)


### Tools and Packages Installation

#### HEAR-CLI
HEAR_CLI installed. See [Guide/HEAR Software/HEAR_CLI/readme.md](./../HEAR%20Software/HEAR%20Software/HEAR_CLI/readme.md).

#### Packages

#### Developer tools

- vscode and extensions
- Yakuake

## Getting Started with Flight Stack Development



### Cloning

### Compilation
#### LeafFC

#### PX4 Autopilot

#### LeafMC

#### HEAR_Msgs

### Sourcing and environment setup

## Getting Started with Petals Stack Development 
TODO: link


## Getting Started with Controller Dashboard Development 
TODO: link

## Getting Started with Web Client Application Development 
TODO: link


# Running the SITL environment


# Debugging Tools
## Debugging MAVLink with Wireshark

# Hardware guides




# Additional Functionalities
## VPN remote access



# TODO REVIEW Getting Started

### Installation
- Software stack installation guide: [Guide/HEAR Software/Operation/SITL/sitl-installation-on-ubuntu20.04.md](./../HEAR%20Software/Operation/SITL/sitl-installation-on-ubuntu20.04.md)
- petal app manager installation guide: [https://droneleaf.github.io/petal-app-manager/getting_started/quickstart.html](https://droneleaf.github.io/petal-app-manager/getting_started/quickstart.html)

## Building Software from Source
### PX4-Autopilot Quick Build

<!-- create bashscript to verify that all  -->
> ```bash
> 

`~/software-stack/PX4-Autopilot` 

1. Install the Python helpers (Yakuake tab name `px4_build` recommended):
   ```bash
   pip3 install kconfiglib
   pip3 install --user jsonschema
   pip3 install --user pyros-genmsg
   pip3 install --user jinja2
   ```
2. Build and launch the default Gazebo classic world:
   ```bash
   cd ~/software-stack/PX4-Autopilot
   make px4_sitl gazebo-classic
   ```
3. Alternate build targets:
   - `make px4_sitl gazebo-classic_dfl` for the DFL model.
   - `HEADLESS=1 make px4_sitl gazebo-classic` to reduce GPU usage.

If the PX4 shell does not show **Ready for takeoff!**, run `pxh> ekf2 start`. Capture terminal logs for troubleshooting.

## Setting up and provisioning
- SITL provisioning guide: [Guide/HEAR Software/Operation/SITL/SITL-drone-provisioning.md](./../HEAR%20Software/Operation/SITL/SITL-drone-provisioning.md)
> for petal app manager to start successfully, make sure that the provisioning steps are completed, open localhost:80 and follow the instructions fully.
## First Run
- Readme guide: [Guide/HEAR Software/Operation/SITL/Readme.md](./../HEAR%20Software/Operation/SITL/Readme.md)

## Contribution and development
- Hear-cli development guide: [Guide/HEAR Software/Development/hear-cli-development-guide.md](../../Development/hear-cli-development-guide.md)
- LeafQGC and Qt tooling guide: [Guide/HEAR Software/Operation/SITL/leafQGC-and-QT-tooling.md](./../HEAR%20Software/Operation/SITL/leafQGC-and-QT-tooling.md)
- leafFC development guide: [Guide/HEAR Software/Operation/SITL/DynamoDB-and-hearfc-debugging.md](./../HEAR%20Software/Operation/SITL/DynamoDB-and-hearfc-debugging.md)
- petal app manager development guide: [https://droneleaf.github.io/petal-app-manager/contributing/contribution_guide.html](https://droneleaf.github.io/petal-app-manager/contributing/contribution_guide.html)

## Wiki contribution guide
