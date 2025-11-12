# DroneLeaf Workspace Topology and Repos Introduction

Purpose: explain the canonical directory structure for DroneLeaf SITL development, list key repositories with locations and purposes.

## Prerequisites
1. HEAR_CLI installed. See [hear-cli installation guide](hear-cli-installation.md).
2. software stack cloned into `~/software-stack`. 
   ```bash
   hear-cli local_machine run_program --p software_stack_clone
   # Choose `dev-sitl` for active development or `main` for the latest stable release
   ```

## Workspace Layout

| Path | Contents | How to get it |
| --- | --- | --- |
| `~/HEAR_CLI` | hear-cli, a centralized management tool and set of codes for HEAR software | Cloned manually (see hear-cli README) |
| `~/software-stack` | All runtime repos (PX4, HEAR_FC, HEAR_Msgs, etc.) | `hear-cli local_machine run_program --p software_stack_clone` |
| `~/LeafMC` [development]| Development source code of Leaf QGroundControl fork | Cloned manually (see leafQGC guide) |
| `~/petal-app-manager-dev` | central directory for petal app manager development. [out of this document scope] | Check petal app manager documentation |


content of software-stack:

| Subdirectory | Purpose |
| --- | --- |
| `PX4-Autopilot` | PX4 firmware and Gazebo SITL simulation |
| `HEAR_FC` | ROS-based flight controller software. Note that actual dev directory is ./HEAR_FC/src/HEAR_FC.|
| `HEAR_Msgs` | ROS message definitions and catkin workspace |
| `HEAR_MC` | Ignored [un maintained] Mission-control / use LeafMC |
| `HEAR_Configurations` | Vehicle, sensor and launch configuration files |
| `HEAR_Docker` | Docker images and build configs for HEAR services |
| `LeafMC` [runtime] | DroneLeaf’s QGroundControl fork source. It is advisable not to modify this directly |
| `mavlink-router` | MAVLink routing/forwarding service |
