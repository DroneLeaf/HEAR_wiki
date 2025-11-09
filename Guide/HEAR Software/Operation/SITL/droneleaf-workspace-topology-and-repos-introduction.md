# DroneLeaf Workspace Topology and Repos Introduction

Understand where each repository lives, how the workspace is organized, and which commands bootstrap the tree. Use this document as the canonical reference for “where things live” on a SITL development machine.

## Workspace Layout

| Path | Contents | Created By |
| --- | --- | --- |
| `~/HEAR_CLI` | hear-cli runner, scripts under `scripts/programs/` | Manual clone per hear-cli README |
| `~/software-stack` | All runtime repos (PX4, HEAR_FC, HEAR_Msgs, etc.) | `hear-cli local_machine run_program --p software_stack_clone` |
| `~/LeafMC` | Leaf QGroundControl fork | Cloned manually (see leafQGC guide) |
| `~/logs` *(optional)* | Collected installer logs (`*.log`) | Manually created |

> `~` refers to the current user’s home directory (default `ubuntu`).

## Preparing the Workspace

1. Follow the hear-cli README to clone and install `HEAR_CLI` under your home directory.
2. Run the software stack clone program (executed later in the SITL installation guide) to populate `~/software-stack`:
   ```bash
   hear-cli local_machine run_program --p software_stack_clone
   ```
   - Choose `dev-sitl` for active development or `main` for the latest stable release.
3. After cloning, verify the expected subdirectories:
   ```bash
   ls ~/software-stack
   # Expected: PX4-Autopilot  HEAR_FC  HEAR_Msgs  HEAR_Mission  scripts  ...
   ```

## Repository Quick Reference

| Repo | Location | Purpose | Key Follow-up Docs |
| --- | --- | --- | --- |
| PX4-Autopilot | `~/software-stack/PX4-Autopilot` | Firmware + Gazebo SITL | (below) |
| HEAR_FC | `~/software-stack/HEAR_FC` | ROS-based flight controller | `DynamoDB-and-hearfc-debugging.md` |
| HEAR_Msgs | `~/software-stack/HEAR_Msgs` | Message definitions + catkin workspace | `DynamoDB-and-hearfc-debugging.md` |
| HEAR_Mission / auxiliary repos | `~/software-stack/*` | Mission management, data tools | Organization-specific docs |
| LeafMC | `~/LeafMC` | DroneLeaf’s QGroundControl fork | `leafQGC-and-QT-tooling.md` |

Ensure each repo uses SSH remotes so internal scripts can push/pull without prompting. If you change locations, update environment variables referenced by hear-cli profiles.

## PX4-Autopilot Quick Build

Once `~/software-stack/PX4-Autopilot` is cloned, complete the initial build to verify toolchains:

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

Next, proceed to [`sitl-installation-on-ubuntu20.04.md`](sitl-installation-on-ubuntu20.04.md) to install the rest of the stack services.
