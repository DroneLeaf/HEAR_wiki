# Overview

This document describes how the SITL is run with PX4 gazebo environment.

## DroneLeaf developers

### Your first SITL run!

As you have your machine ready for development and testing, let's run the SITL setup to play around the software stack.

#### One-time setup

Run the following `hear-cli` commands **in order** on your local machine:

```bash
hear-cli local_machine run_program --p hear_docker_clone
hear-cli local_machine run_program --p hear_docker_sitl_full_system_install
hear-cli local_machine run_program --p install_system_dependencies_sitl
hear-cli local_machine run_program --p docker_install
hear-cli local_machine run_program --p node_install
hear-cli local_machine run_program --p configure_software_setup_autostart_arm
hear-cli local_machine run_program --p init_ecr_pull_profile
hear-cli local_machine run_program --p init_sync_profile
hear-cli local_machine run_program --p data_lifecycle_prepare
hear-cli local_machine run_program --p controller_dashboard_prepare
hear-cli local_machine run_program --p software_stack_clone 
#Note: Choose the dev-sitl branch if you want to install a development environment, otherwise choose main branch [as of Oct 2025, main is way behind. Use dev for stable release, or dev-sitl for development].
hear-cli local_machine run_program --p set_fc_configs
hear-cli local_machine run_program --p petal_app_manager_prepare_sitl
```

Logout and log back in after completing the setup to ensure all autostart settings are applied.

---
#### 1 - Mavlink Router


Verify the Mavlink router is running:
```bash
sudo systemctl status mavlink-router.service
```


#### 2 - PX4-Autopilot

Before running PX4-Autopilot download the following dependencies: 
```bash
pip3 install kconfiglib
pip3 install --user jsonschema
pip3 install --user pyros-genmsg
pip3 install --user jinja2
```
Following:

```bash
cd ~/software-stack/PX4-Autopilot
make px4_sitl gazebo-classic
```
This will launch a gazeboo world that contains a drone instance, along with the px4 firmware running in SITL setup for this drone.

If you want to launch a specific drone instance (e.g. DFL), you can specify it by adding the instance postfix after the environment name as follows

```bash
make px4_sitl gazebo-classic_dfl
```

If you do not see the green colored "Ready for takeoff!" message in the terminal, you can use the following command.
```bash
pxh> ekf2 start
```

**Note**: Sometimes you want to run the SITL environment without the UI. For that use:
```bash
HEADLESS=1 make px4_sitl gazebo-classic
```


#### 3 - Leaf QGroundControl

Download QGC from the software-stack repo release page.


Run the QGroundControl and connect to the PX4 SITL instance via MAVLink.

```bash
cd ~/Downloads # where you downloaded the QGroundControl AppImage
./LeafMC.AppImage
```
You should see something similar to this:

![QGroundControl](media/LeafMCQGC_PX4SITL.png)

If you see the message "FC Disconnected" like the screenshot then you can move on and launch the FC. Otherwise, if you see the message "Disconnected", you need to create a comm link from QGC settings to be able to communicate through MAVLink.

Click on the QGC symbol on the top left corner of the window and open up the application settings.
![QGroundControl](media/QGC_setting.png)

Go to Comm Links tab on the top left menu and click on "Add". Then, fill the settings as follows and create the comm link. You can put anything you want in the place of "COMMLINK-NAME".
![QGroundControl](media/commlink.png)

After that, you can connect to the comm link instance you have created (TCP://:5760), in this case the comm link name is set to "SITL".
![QGroundControl](media/connect_commlink.png)

#### 4 - Make sure drone data is synced locally correctly

Make sure you are registered in `fly.droneleaf.io` and that your drone is bound to a license.

**To do this:**
Follow the steps in the flowchart located in the HEAR Wiki:
`Guide > Hardware and Process > Commissioning > 2 Ready for FSAC > Ready for FSAC Updated.draw.io`

The registration and licensing steps are located toward the **end** of the flowchart.

After completing registration:

1. Navigate to `~/software-stack/HEAR_Msgs` and compile:

   ```bash
   cd ~/software-stack/HEAR_Msgs
   catkin_make
   source devel/setup.bash
   ```

2. Navigate to `~/software-stack/HEAR_FC` and compile the FC with SITL target:

   ```bash
   cd ~/software-stack/HEAR_FC
   catkin_make -DCMAKE_BUILD_TYPE=Debug -DHEAR_TARGET=SITL
   source devel/setup.bash
   ```

3. Launch the Flight Controller:

   ```bash
   roslaunch flight_controller px4_flight_mavlink_opti_onboard_mission.launch
   ```

You should see a **green bar** and the message: **PX4 Ready To Fly**

---

### The SITL simulation is initialized and ready to fly. 🚀

---
