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

## Prerequisites

- Completed `sitl-installation-on-ubuntu20.04.md`
- `env.zip` certificate bundle supplied by DroneLeaf support
- hear-cli authenticated with GitHub token

## 1. Stage Environment Certificates

Request the latest `env.zip` from DroneLeaf Support (Ahmed Hashem). Place it in your home directory and run the following in a Yakuake tab named `hear_docker_env_setup`:

```bash
[ -f "./env.zip" ] && unzip -o "./env.zip" -d ./env || echo "env.zip not found"
mkdir -p "$HOME/HEAR_CLI/scripts/programs/init_ecr_pull_profile" \
         "$HOME/HEAR_CLI/scripts/programs/init_sync_profile"
if [ -f ./env/init_ecr_pull_profile/env ]; then
  cp -v ./env/init_ecr_pull_profile/env "$HOME/HEAR_CLI/scripts/programs/init_ecr_pull_profile/env"
else
  echo "Warning: ./env/init_ecr_pull_profile/env not found"
fi
if [ -f ./env/init_sync_profile/env ]; then
  cp -v ./env/init_sync_profile/env "$HOME/HEAR_CLI/scripts/programs/init_sync_profile/env"
else
  echo "Warning: ./env/init_sync_profile/env not found"
fi
```

## 2. Initialize hear-cli Profiles

```bash
hear-cli local_machine run_program --p init_ecr_pull_profile
hear-cli local_machine run_program --p init_sync_profile
hear-cli local_machine run_program --p data_lifecycle_prepare
sudo reboot
```

### Post-reboot

```bash
hear-cli local_machine run_program --p controller_dashboard_prepare
hear-cli local_machine run_program --p software_stack_clone
# Choose branch: dev-sitl for development, main for latest stable release.
hear-cli local_machine run_program --p set_fc_configs
```

Optional (but recommended for app testing):

```bash
hear-cli local_machine run_program --p petal_app_manager_prepare_sitl
```

Reboot once more to ensure services pick up new configs:

```bash
sudo reboot
```

## 3. Readiness Checklist

- `docker ps` shows the DroneLeaf containers running.
- `systemctl status mavlink-router.service` reports **active (running)**.
- hear-cli profile directories (`~/HEAR_CLI/scripts/programs/*/env`) contain the staged certificates.
- `~/software-stack` contains the cloned repositories from the chosen branch.


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
