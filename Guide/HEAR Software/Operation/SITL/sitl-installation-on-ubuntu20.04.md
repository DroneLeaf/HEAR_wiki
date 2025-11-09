# SITL Installation on Ubuntu 20.04

Run the scripted installers that prepare Docker, system dependencies, and the DroneLeaf SITL services on a freshly provisioned workstation.

## Prerequisites

- Base OS completed per `development-machine-OS-installation-for-droneleaf-stack.md`
- Toolchain prerequisites installed per `recommended-tools-and-common-practices.md`
- Workspace prepared and `HEAR_CLI` cloned (see `droneleaf-workspace-topology-and-repos-introduction.md`)
- GitHub token cached (hear-cli commands prompt only once)

> Always execute commands from Yakuake tabs named after the step (e.g., `hear_docker_clone`). Capture output with `tee` whenever possible.

## Step 1 – Clone Docker Assets

```bash
hear-cli local_machine run_program --p hear_docker_clone
```
- When prompted, enter your GitHub username and personal access token.

## Step 2 – Install the SITL Full System

```bash
hear-cli local_machine run_program --p hear_docker_sitl_full_system_install | tee docker_sitl_full_system.log
```
- Log file: `~/docker_sitl_full_system.log`
- Reboot immediately afterwards:
  ```bash
  sudo reboot
  ```

## Step 3 – System Dependencies for SITL

After reboot, open a tab named `hear_docker_setup`:

```bash
hear-cli local_machine run_program --p install_system_dependencies_sitl | tee install_system_dependencies_sitl.log
sudo reboot
```

## Step 4 – Docker & Node Tooling Cleanup

Tab name: `hear_docker_cleanup`

```bash
hear-cli local_machine run_program --p docker_install | tee docker_install.log
hear-cli local_machine run_program --p node_install | tee node_install.log
hear-cli local_machine run_program --p configure_software_setup_autostart_sitl | tee configure_software_setup_autostart_sitl.log
sudo reboot
```

## Post-Install Quick Checks

After the final reboot:

```bash
docker ps
systemctl status mavlink-router.service
```

You should see the DroneLeaf containers running and `mavlink-router.service` in the **active (running)** state.

Continue with [`SITL-drone-provisioning.md`](SITL-drone-provisioning.md) to import environment certificates, initialize hear-cli profiles, and register the SITL node.
