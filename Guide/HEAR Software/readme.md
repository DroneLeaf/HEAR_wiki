# DroneLeaf Software Stack

A minimal list of the software components used in the SITL environment.

Components (all are connected to each other):
- hear-cli
- software stack:
    - mavlink router
    - PX4 Autopilot
    - HEAR_FC
    - HEAR_Msgs
    - LeafMC (LeafQGC)
    - controller dashboard
    - on-board express API
    - proxy server
    - dynamodb
    - docker
- petal app manager:
    - petal app manager
    - petals:
      - petal-pymavlink
      - petal-leafsdk
      - etc.

# Getting Started
## Prerequisites
- Ubuntu 20.04 LTS
- Hardware: minimum 16GB RAM, 4-core CPU, 256GB free disk space
- fresh installation [recommended]
For full details, see the OS provisioning guide: [Guide/HEAR Software/Operation/SITL/os-provisioning-ubuntu20.04.md](os-provisioning-ubuntu20.04.md)
## Installation
- hear-cli installation guide: [Guide/HEAR Software/Operation/SITL/hear-cli-installation.md](hear-cli-installation.md)
- Software stack installation guide: [Guide/HEAR Software/Operation/SITL/sitl-installation-on-ubuntu20.04.md](sitl-installation-on-ubuntu20.04.md)
- petal app manager installation guide: [https://droneleaf.github.io/petal-app-manager/getting_started/quickstart.html](https://droneleaf.github.io/petal-app-manager/getting_started/quickstart.html)
> for petal app manager to start successfully, make sure that the provisioning steps are completed, open localhost:80 and follow the instructions fully.
## Setting up and provisioning
- SITL provisioning guide: [Guide/HEAR Software/Operation/SITL/SITL-drone-provisioning.md](SITL-drone-provisioning.md)
## First Run
- Readme guide: [Guide/HEAR Software/Operation/SITL/Readme.md](Readme.md)

## Contribution and development
- Hear-cli development guide: [Guide/HEAR Software/Development/hear-cli-development-guide.md](../../Development/hear-cli-development-guide.md)
- LeafQGC and Qt tooling guide: [Guide/HEAR Software/Operation/SITL/leafQGC-and-QT-tooling.md](leafQGC-and-QT-tooling.md)
- leafFC development guide: [Guide/HEAR Software/Operation/SITL/DynamoDB-and-hearfc-debugging.md](DynamoDB-and-hearfc-debugging.md)
- petal app manager development guide: [https://droneleaf.github.io/petal-app-manager/contributing/contribution_guide.html](https://droneleaf.github.io/petal-app-manager/contributing/contribution_guide.html)