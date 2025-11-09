# SITL Drone Provisioning

After Docker and system dependencies are installed, provision the SITL node so it can pull private images, sync data, and appear as a registered drone in DroneLeaf services.

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

## 4. Licensing and Data Sync

To finish provisioning, bind the SITL node to a drone license and sync DynamoDB data:

1. Register on <https://fly.droneleaf.io> and bind the SITL hardware to the assigned license following the flowchart in `Guide/Hardware and Process/Commissioning/2 Ready for FSAC/Ready for FSAC Updated.draw.io`.
2. Once licensed, proceed to [`DynamoDB-and-hearfc-debugging.md`](DynamoDB-and-hearfc-debugging.md) to run the DynamoDB sync, build `HEAR_Msgs`, and prepare the HEAR_FC workspace.

At this point the SITL workstation is recognized as a drone and ready for functional testing.
