# SITL Installation and Preparation on a New Machine

This index replaces the old monolithic guide. Follow the documents below in order when preparing a fresh SITL workstation.

1. [`development-machine-OS-installation-for-droneleaf-stack.md`](development-machine-OS-installation-for-droneleaf-stack.md)  
   Provision Ubuntu 20.04 LTS, configure the hostname, swap, region settings, and ensure hardware readiness.
2. [`recommended-tools-and-common-practices.md`](recommended-tools-and-common-practices.md)  
   Install the standard toolchain (Yakuake profiles, apt packages, Git/GitHub configuration, logging conventions).
3. [`droneleaf-workspace-topology-and-repos-introduction.md`](droneleaf-workspace-topology-and-repos-introduction.md)  
   Learn the canonical `~/software-stack` layout, clone hear-cli, and run the initial PX4 build checks.
4. [`sitl-installation-on-ubuntu20.04.md`](sitl-installation-on-ubuntu20.04.md)  
   Execute the hear-cli automation (`hear_docker_clone`, `hear_docker_sitl_full_system_install`, dependency installers) and verify Docker/systemd services.
5. [`SITL-drone-provisioning.md`](SITL-drone-provisioning.md)  
   Import environment certificates, initialize hear-cli profiles, run controller dashboard prep, and confirm the node registers as a SITL drone.
6. [`DynamoDB-and-hearfc-debugging.md`](DynamoDB-and-hearfc-debugging.md)  
   Sync DynamoDB data, build `HEAR_Msgs`, compile `HEAR_FC`, and follow the debugging workflow in VS Code.
7. [`leafQGC-and-QT-tooling.md`](leafQGC-and-QT-tooling.md)  
   Prepare the LeafMC repo, install Qt 5.15.2 + Qt Creator, and solve common GUI tooling issues.

Refer back to this index whenever you need to reset a workstation or locate the appropriate runbook for a given task.
