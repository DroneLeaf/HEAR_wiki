# SITL Installation on Ubuntu 20.04

Run the scripted installers that prepare Docker, system dependencies, and the DroneLeaf SITL services on a freshly provisioned workstation.

## Prerequisites

- Base OS completed per `development-machine-OS-installation-for-droneleaf-stack.md`
- Toolchain prerequisites installed per `recommended-tools-and-common-practices.md`
- Workspace prepared and `HEAR_CLI` cloned (see `droneleaf-workspace-topology-and-repos-introduction.md`)
- GitHub token cached (hear-cli commands prompt only once)

> Always execute commands from Yakuake. Capture output with `tee` whenever possible. [Refer to recommended-tools-and-common-practices.md for logging conventions.]

## Step 0 – Install hear-cli (if not done) and required dependencies
- Check if `hear-cli` is already installed:

  ```bash
  hear-cli --version
  ```
  If the command is not found, check out `droneleaf-workspace-topology-and-repos-introduction.md` to clone and install `HEAR_CLI`.

- make sure that the following dependencies are installed:
  apt packages:

  ```bash
    sudo apt install -y build-essential libdbus-glib-1-dev libgirepository1.0-dev \
    git curl wget cmake unzip pkg-config libssl-dev libjpeg-dev libpng-dev \
    libtiff-dev libusb-1.0-0-dev python3-pip jq
  ```
  python3 packages:

  ```bash
    pip3 install --force-reinstall ninja
    pip3 install testresources
    pip3 install kconfiglib
    pip3 install --user jsonschema
    pip3 install --user pyros-genmsg
    pip3 install --user jinja2
  ```
## Step 1 – Clone Docker Assets and install

```bash
hear-cli local_machine run_program --p hear_docker_clone | tee step1_docker_clone.log
```
- When prompted, enter your GitHub username and personal access token.

## Step 2 – Install the SITL Full System

```bash
hear-cli local_machine run_program --p hear_docker_sitl_full_system_install | tee step2_docker_sitl_full_system.log
```
- check log and then reboot the machine

## Step 3 – System Dependencies for SITL

```bash
hear-cli local_machine run_program --p install_system_dependencies_sitl | tee step3_install_system_dependencies_sitl.log
```
- check log and then reboot the machine

## Step 4 – Docker & Node Tooling Cleanup

Tab name: `hear_docker_cleanup`

```bash
hear-cli local_machine run_program --p docker_install | tee docker_install.log
hear-cli local_machine run_program --p node_install | tee node_install.log
hear-cli local_machine run_program --p configure_software_setup_autostart_sitl | tee configure_software_setup_autostart_sitl.log
sudo reboot
```

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

## Post-Install Quick Checks

After the final reboot:

```bash
docker ps
systemctl status mavlink-router.service
```
Prepare your wireshark to capture on the `mavlink-router` interface to monitor MAVLink traffic.
```bash
sudo apt install -y wireshark-qt
hear-cli local_machine run_program --p mavlink_update_wireshark_plugin
```

You should see the DroneLeaf containers running and `mavlink-router.service` in the **active (running)** state.

Continue with [`SITL-drone-provisioning.md`](SITL-drone-provisioning.md) to import environment certificates, initialize hear-cli profiles, and register the SITL node.
