# DynamoDB and HEAR_FC Debugging

Use this guide to synchronize cloud data, register the SITL instance as a licensed drone, and build/debug the HEAR_FC stack.

## Prerequisites

- SITL workstation provisioned and cloned (`SITL-drone-provisioning.md`)
- Access to <https://fly.droneleaf.io>
- ROS and catkin packages provided by the SITL installers

## 1. Licensing and Cloud Registration

Follow the flowchart located at `Guide/Hardware and Process/Commissioning/2 Ready for FSAC/Ready for FSAC Updated.draw.io`. The final stages of that flow bind the SITL node to a DroneLeaf license on `fly.droneleaf.io`. Confirm the license status before pulling data.

## 2. DynamoDB Sync (HEAR_Msgs)

Once the license is active:

```bash
cd ~/software-stack/HEAR_Msgs
catkin_make
source devel/setup.bash
```

Re-run these commands whenever message definitions change or after cleaning the workspace.

## 3. Build the HEAR_FC Target

```bash
cd ~/software-stack/HEAR_FC
catkin_make -DCMAKE_BUILD_TYPE=Debug -DHEAR_TARGET=SITL
source devel/setup.bash
```

### Clean Rebuild

```bash
cd ~/software-stack/HEAR_FC
rm -rf devel/ build/
catkin_make -DCMAKE_BUILD_TYPE=Debug -DHEAR_TARGET=SITL
source devel/setup.bash
```

## 4. Launching the Flight Controller

```bash
roslaunch flight_controller px4_flight_mavlink_opti_onboard_mission.launch
```

Expect to see a green status bar with **PX4 Ready To Fly** once MAVLink connectivity is established.

## 5. Debugging Workflow (VS Code)

1. Open VS Code in `~/software-stack/HEAR_FC/src/HEAR_FC`.
2. Navigate to **Run and Debug**.
3. Use the `ROS: Launch Dbg px4_flight_mavlink_opti_onboard_mission` configuration:
   ```json
   {
       "version": "0.2.0",
       "console": "integratedTerminal",
       "configurations": [
           {
               "name": "ROS: Launch Dbg px4_flight_mavlink_opti_onboard_mission",
               "request": "launch",
               "target": "${workspaceFolder}/Flight_controller/launch/scratch/px4_flight_mavlink_opti_onboard_mission.launch",
               "launch": [
                   "rviz",
                   "gz",
                   "gzclient",
                   "gzserver"
               ],
               "type": "ros"
           }
       ]
   }
   ```
4. Click the green play button to attach the debugger. Monitor RViz/Gazebo output for sensor states and confirm topics with `rosnode list` / `rostopic echo`.

## Troubleshooting Tips

- Run `hear-cli local_machine run_program --p init_sync_profile` again if DynamoDB data looks stale.
- Use `journalctl -u mavlink-router.service -f` to monitor link health while debugging.
- Keep `catkin_make` logs under `~/logs` to share with the team during incident reviews.
