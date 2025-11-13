# Prepare Data Life Cycle Management for SITL environment

<!-- load the svg image -->
![Data Lifecycle Management](./Data_lifecycle_management.drawio.svg)

## Pre-requisites Installation

**Pull the latest `HEAR_CLI`**

All steps will be done using `hear-cli`
- Docker Install (`Run Once on target`)

```bash
hear-cli local_machine run_program --p docker_install
```

- Node.js Install (`Run Once on target`)

```bash
hear-cli local_machine run_program --p node_install
```
 **Reboot your system**.


- Init Ecr Pull Profile

```bash
hear-cli local_machine run_program --p init_ecr_pull_profile
```

- Init Sync Profile

```bash
hear-cli local_machine run_program --p init_sync_profile
```

- Data lifecycle prepare

```bash
hear-cli local_machine run_program --p data_lifecycle_prepare
```

- Controller Dashboard prepare

```bash
hear-cli local_machine run_program --p controller_dashboard_prepare
```

- Set FC Configs

```bash
hear-cli local_machine run_program --p set_fc_configs
```

```bash
hear-cli local_machine run_program --p install_system_dependencies_sitl
```


```bash
hear-cli local_machine run_program --p configure_software_setup_autostart_sitl
```



## How to run

- open the `controller dashboard`: http://localhost in browser
- copy the `Machine ID` from `Device Information` section


- open `client app` types page: https://fly.droneleaf.io/client/drone-types
- *Only if Required:* Press `Add Drone Type` Button and complete the process

- open `client app` drones page : https://fly.droneleaf.io/client/drones

- Press `Add Drone` Button, paste the `Machine ID` you copied from the controller dashboard, select the type you created before then complete the process.

- Go to the drone instance you added.

- Navigate to the `License` tab, and use promo code `devop` to license the drone.

- Navigate to `Settings` Tab and enter the right settings for your drone.

- Navigate to the `License` tab again and generate the API key. Copy the key and paste it in the controller dashboard under the `Access and Secret Keys` section.

- Go to the `Syncing` page in the `controller dashboard` and press `Start` button under the `Cloud To Local Sync` card, wait until finishing the process.

- Now you are ready to use dynamodb in hear-fc

## Notes:

Online configs filter search is limited in number of results

Add hear-cli command to install mavlink-router and roscore services for SITL