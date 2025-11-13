# DynamoDB and HEAR_FC Debugging

Use this guide to synchronize cloud data, register the SITL instance as a licensed drone, and build/debug the HEAR_FC stack.

## Prerequisites

- SITL installation completed (`sitl-installation-on-ubuntu20.04.md`)
- Commissioning completed fully on <https://fly.droneleaf.io>

> Note: if you want to debug the leafFC, make sure to disable the service so it does not conflict:
```bash
sudo systemctl stop leafFC.service
sudo systemctl disable leafFC.service
```     

## 2. DynamoDB Sync 

If your DynamoDB data is stale or missing, you would face issues when launching HEAR_FC. To sync the local DynamoDB instance with the cloud data, open the controller dashboard (ip address of the edge device, e.g., http://localhost) -> Syncing -> Cloud to Local Sync -> Start Sync.


## 2. HEAR_Msgs Building
Re-run these commands whenever message definitions change or after cleaning the workspace.

```bash
cd ~/software-stack/HEAR_Msgs
catkin_make
source devel/setup.bash
```


## 3. Build the HEAR_FC Target, and Launch
Hear_FC is dependent on HEAR_Msgs, so ensure that step 2 is completed first. Then run:

```bash
cd ~/software-stack/HEAR_FC
rm -rf devel/ build/  # optional: clean build
catkin_make -DCMAKE_BUILD_TYPE=Debug -DHEAR_TARGET=SITL
source devel/setup.bash
# Launching the flight controller 
roslaunch flight_controller px4_flight_mavlink_opti_onboard_mission.launch
```

if your LeafMC is running and connected to the PX4, expect to see a green status bar with **PX4 Ready To Fly**.

## 4. Debugging Workflow (VS Code)

1. Open VS Code in `~/software-stack/HEAR_FC/src/HEAR_FC`.
2. Navigate to **Run and Debug**.
3. Use the `ROS: Launch Dbg px4_flight_mavlink_opti_onboard_mission` configuration [it is inluded in the repo as `.vscode/launch.json`]

## Troubleshooting Tips

- Run `hear-cli local_machine run_program --p init_sync_profile` again if DynamoDB data looks stale.
- Use `journalctl -u mavlink-router.service -f` to monitor link health while debugging.

### Tips
- Syncing DynamoDB from cloud to local would overwrite any local changes. Local changes are not recommended.
- Some DynamoDB tables require JSON and Array formatting. In case of suspicious data, verify with DroneLeaf support.
- If a table is missing, redo the sync process, check the provisioning steps, or contact DroneLeaf support.
