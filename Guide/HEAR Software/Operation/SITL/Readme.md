# Overview

Use this entry point to navigate the SITL documentation set and launch your first DroneLeaf simulation.

## Setup Pathway

Follow the runbooks in order:

1. [`development-machine-OS-installation-for-droneleaf-stack.md`](development-machine-OS-installation-for-droneleaf-stack.md)
2. [`recommended-tools-and-common-practices.md`](recommended-tools-and-common-practices.md)
3. [`droneleaf-workspace-topology-and-repos-introduction.md`](droneleaf-workspace-topology-and-repos-introduction.md)
4. [`sitl-installation-on-ubuntu20.04.md`](sitl-installation-on-ubuntu20.04.md)
5. [`SITL-drone-provisioning.md`](SITL-drone-provisioning.md)
6. [`DynamoDB-and-hearfc-debugging.md`](DynamoDB-and-hearfc-debugging.md)
7. [`leafQGC-and-QT-tooling.md`](leafQGC-and-QT-tooling.md)

Each guide is scoped to a single topic (OS install, tooling, workspace layout, hear-cli installers, provisioning, data/debug, UI tooling) so updates remain isolated.

## Your First SITL Run

After completing the provisioning documents:

1. **Verify services**  
   ```bash
   docker ps
      systemctl status mavlink-router.service
   ```
   Both commands should show active DroneLeaf components.

2. **Build & launch PX4 (Gazebo Classic)**  
   ```bash
   cd ~/software-stack/PX4-Autopilot
   pip3 install kconfiglib
   pip3 install --user jsonschema pyros-genmsg jinja2
   make px4_sitl gazebo-classic
   # Alt: make px4_sitl gazebo-classic_dfl, or HEADLESS=1 ...
   ```
   If the shell does not print “Ready for takeoff!”, use `pxh> ekf2 start`.

3. **Launch LeafQGC (LeafMC)** – follow [`leafQGC-and-QT-tooling.md`](leafQGC-and-QT-tooling.md) to configure Qt Creator or run the AppImage. Connect to `TCP://:5760` via the Comm Links panel if QGC does not auto-connect.

4. **Sync DynamoDB and start HEAR_FC** – see [`DynamoDB-and-hearfc-debugging.md`](DynamoDB-and-hearfc-debugging.md) for catkin build commands and VS Code debugging.

You should see the **PX4 Ready To Fly** banner in QGC once all components are connected. Happy sim flying! 🚀
