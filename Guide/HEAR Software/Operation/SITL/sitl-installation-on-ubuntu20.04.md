# SITL Installation on Ubuntu 20.04

Run the scripted installers that prepare all requirements for SITL operation on a fresh Ubuntu 20.04 LTS machine.

## Prerequisites
- Ubuntu 20.04 LTS Workspace prepared [droneleaf-workspace-topology-and-repos-introduction.md](droneleaf-workspace-topology-and-repos-introduction.md)
- `HEAR_CLI` installed (see [Guide/HEAR Software/HEAR_CLI/readme.md](./../HEAR%20Software/HEAR%20Software/HEAR_CLI/readme.md))
- GitHub token [classic token with repo access]
- Get `env.zip` from DroneLeaf Support (Ahmed Hashem) but do not unzip it yet. A script in a later step will handle this. Run terminal from the directory at which the file is located. [as of Nov 2025]

## Hints for getting installation support
> Capture output with `tee` whenever possible. [Refer to [recommended-tools-apps-and-extensions.md](./recommended-tools-apps-and-extensions.md) for logging conventions.]

## 0. Dependencies Installation

- Install :
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

## 1. Environment Certificates

Request the latest `env.zip` from DroneLeaf Support (Ahmed Hashem). Run the following script from same directory where `env.zip` is located.

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

## 2. Hear-Cli based installation scripts.

> When prompted, enter your GitHub username and personal access token.

```bash
hear-cli local_machine run_program --p hear_docker_clone | tee docker_clone.log
hear-cli local_machine run_program --p hear_docker_sitl_full_system_install | tee step2_docker_sitl_full_system.log
sudo reboot
```

```bash
hear-cli local_machine run_program --p install_system_dependencies_sitl | tee step3_install_system_dependencies_sitl.log
hear-cli local_machine run_program --p docker_install | tee docker_install.log
hear-cli local_machine run_program --p node_install | tee node_install.log
sudo reboot
```

```bash
hear-cli local_machine run_program --p configure_software_setup_autostart_sitl | tee configure_software_setup_autostart_sitl.log
sudo reboot
```

```bash
hear-cli local_machine run_program --p init_ecr_pull_profile
hear-cli local_machine run_program --p init_sync_profile
hear-cli local_machine run_program --p data_lifecycle_prepare
sudo reboot
```

```bash
hear-cli local_machine run_program --p controller_dashboard_prepare
hear-cli local_machine run_program --p software_stack_clone
# Choose branch: dev-sitl for development, main for latest stable release.
hear-cli local_machine run_program --p set_fc_configs
```

```bash
hear-cli local_machine run_program --p petal_app_manager_prepare_sitl
sudo reboot
```

> Read more about Petal App Manager in the [Petal App Manager Documentation](https://droneleaf.github.io/petal-app-manager/).

## 3. Readiness Checklist

```bash
docker ps
# Expected output: multiple containers running
systemctl status mavlink-router.service  --no-pager
# Expected output: active (running)
sudo systemctl status Controller_Dashboard-app.service --no-pager
# Expected output: active (exited)
sudo systemctl status Controller_Dashboard_express_api_server_app.service --no-pager
# Expected output: active (running)
```

Web access:
- Open a browser and navigate to `http://localhost` to access the Controller Dashboard.
- 
