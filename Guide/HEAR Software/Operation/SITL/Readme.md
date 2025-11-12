# Overview

Use this entry point to navigate the SITL documentation set and launch your first DroneLeaf simulation.

## Setup Pathway

Follow the runbooks in order:

1. [`development-machine-OS-installation-for-droneleaf-stack.md`](development-machine-OS-installation-for-droneleaf-stack.md)  
   Provision Ubuntu 20.04 LTS, configure hostname, swap, region settings, and ensure hardware readiness.
2. [`recommended-tools-apps-and-extensions.md`](recommended-tools-apps-and-extensions.md)
   Install the standard toolchain (Yakuake profiles, apt packages, Git/GitHub configuration, logging conventions).
3. [`droneleaf-workspace-topology-and-repos-introduction.md`](droneleaf-workspace-topology-and-repos-introduction.md)  
   Learn the canonical `~/software-stack` layout, clone hear-cli, and run the initial PX4 build checks.
4. [`sitl-installation-on-ubuntu20.04.md`](sitl-installation-on-ubuntu20.04.md)  
   Execute hear-cli automation (`hear_docker_clone`, `hear_docker_sitl_full_system_install`), run dependency installers, and verify Docker/systemd services.
5. [`SITL-drone-provisioning.md`](SITL-drone-provisioning.md)  
   Import environment certificates, initialize hear-cli profiles, prepare the controller dashboard, and confirm the node registers as a SITL drone.
6. [`DynamoDB-and-hearfc-debugging.md`](DynamoDB-and-hearfc-debugging.md)  
   Sync DynamoDB data, build `HEAR_Msgs`, compile `HEAR_FC`, and follow the VS Code debugging workflow.
7. [`leafQGC-and-QT-tooling.md`](leafQGC-and-QT-tooling.md)  
   Prepare the LeafMC repo, install Qt 5.15.2 + Qt Creator, and resolve common GUI tooling issues.

Each guide is scoped to a single topic (OS install, tooling, workspace layout, hear-cli installers, provisioning, data/debug, UI tooling) so updates remain isolated.

## Your First SITL Run

After completing the provisioning documents:

1. **Verify services**  
   ```bash
   docker ps
   systemctl status mavlink-router.service
   ```
   Both commands should show active DroneLeaf components.

   Example `docker ps` output:
   ```bash
   CONTAINER ID   IMAGE                                                                       COMMAND                  CREATED       STATUS       PORTS                                                                              NAMES
   8659d0cd0b1f   296257236984.dkr.ecr.me-south-1.amazonaws.com/controller_dashboard:latest   "/docker-entrypoint.…"   5 days ago    Up 7 hours   0.0.0.0:80->80/tcp, [::]:80->80/tcp, 0.0.0.0:4201->4201/tcp, [::]:4201->4201/tcp   controller_dashboard_app
   d428784d5ff9   296257236984.dkr.ecr.me-south-1.amazonaws.com/on_board_express_api:latest   "docker-entrypoint.s…"   3 weeks ago   Up 7 hours   0.0.0.0:3000->3000/tcp, [::]:3000->3000/tcp, 8000/tcp                              express_local
   9f0dd9502d7c   296257236984.dkr.ecr.me-south-1.amazonaws.com/dynamodb_local:latest         "java -Djava.library…"   3 weeks ago   Up 7 hours   0.0.0.0:8000->8000/tcp, [::]:8000->8000/tcp                                        dynamodb-local
   b75c89b947fe   296257236984.dkr.ecr.me-south-1.amazonaws.com/proxy_server:latest           "docker-entrypoint.s…"   3 weeks ago   Up 7 hours   0.0.0.0:4000->4000/tcp, [::]:4000->4000/tcp                                        proxy_server
   a8110e744da0   296257236984.dkr.ecr.me-south-1.amazonaws.com/dynamodb_manager:latest       "/docker-entrypoint.…"   3 weeks ago   Up 7 hours   8080/tcp, 0.0.0.0:8080->80/tcp, [::]:8080->80/tcp                                  dynamodb_manager
   ```

## Here is a reminder of how to launch the full SITL stack following successful installation and provisioning:
2. **Build & launch PX4 (Gazebo Classic)**  
   ```bash
   cd ~/software-stack/PX4-Autopilot
   pip3 install kconfiglib
   pip3 install --user jsonschema pyros-genmsg jinja2
   make px4_sitl gazebo-classic
   # Alt: make px4_sitl gazebo-classic_dfl, or HEADLESS=1 ...
   ```
   If the shell does not print “Ready for takeoff!”, use `pxh> ekf2 start`.

   > For increased visibility in Gazebo, `View -> Wireframe`.' 


3. **Launch LeafQGC (LeafMC)** – follow [`leafQGC-and-QT-tooling.md`](leafQGC-and-QT-tooling.md) to configure Qt Creator or run the AppImage. Connect to `TCP://:5760` via the Comm Links panel if QGC does not auto-connect.

   > reffer to [`leafQGC-and-QT-tooling.md`](leafQGC-and-QT-tooling.md) for note regarding common connection issues.

4. **Sync DynamoDB and start HEAR_FC** – see [`DynamoDB-and-hearfc-debugging.md`](DynamoDB-and-hearfc-debugging.md) for catkin build commands and VS Code debugging.
 
   You should see the **PX4 Ready To Fly** banner in QGC once all components are connected. Happy sim flying! 


## Common issues and troubleshooting 
 This section is covered in the individual guides above. Refer back to this index to locate the appropriate runbook for a given task.