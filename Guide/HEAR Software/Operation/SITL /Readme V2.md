### Installation Steps for Ubuntu 20.04 LTS
1. Backup any important data on your computer before proceeding with the installation, as it may involve formatting your hard drive. If you plan to dual-boot with another operating system, ensure you have enough free space on your hard drive. We need a partition with at least 128 GB of free space for drone leaf software stack compilation and testing. We recommend at least 256 GB of free space. if there is no empty partition, you can shrink an existing partition using a windows built-in Disk Management tool: https://help.ubuntu.com/community/HowtoResizeWindowsPartitions
2. Download ubuntu 20.04 iso from official website, and Create a bootable usb using rufus or etcher: https://ubuntu.com/tutorials/create-a-usb-stick-on-ubuntu#1-overview
3. Insert the usb into the target machine and boot from it (you may need to change the boot order in BIOS/UEFI settings):
    - Restart the computer and press the appropriate key (usually F2, F12, DEL, or ESC) to enter BIOS/UEFI settings.
    - Navigate to the boot menu and set the USB drive as the first boot device.
    - Save changes and exit BIOS/UEFI settings. The computer should now boot from the USB drive.    
4. Select "Try Ubuntu" to boot into the live environment before installing it on your hard drive.
5. Make sure you have a stable internet connection during the installation process. Make sure to connect to Wi-Fi or plug in an Ethernet cable. Notes:
    - If your wifi card is not recognized, you may need to install additional drivers after installation. This is known for internal Broadcom wifi cards of 10th gen and later Intel laptops. To continue: use an Ethernet connection or a USB wifi adapter that is recognized out of the box.
    - After installation, do a kernel upgrade (unofficially) to get the latest drivers for your hardware.
6. Double-click the "Install Ubuntu" icon on the desktop to start the installation process.
7. Follow the on-screen instructions to complete the installation:
    - Select your language and keyboard layout.
    - Choose "Normal installation" and check the boxes for "Download updates while installing Ubuntu" and "Install third-party software for graphics and Wi-Fi hardware and additional media formats."
    - To install Ubuntu alongside an existing operating system: first, you need to shrink the existing partition using a partition manager (like GParted) from the live environment. Then select "Install Ubuntu alongside [your existing OS]" during the installation process. Alernatively, boot back into windows and use the built-in Disk Management tool to shrink the partition before starting the Ubuntu installation. [you need more than 128 GB of free space for drone leaf software stack compilation and testing. we recommend at least 256 GB of free space]
    - If you want to erase the entire disk and install Ubuntu, select "Erase disk and install Ubuntu." (This will delete all data on the disk.)
8. Set your time zone, create a user account, and set a password. [use default username "ubuntu" for easier reference in documentation, and make sure to remember the password you set here]
9. Wait for the installation to complete. This may take some time, depending on your internet speed and system performance.
10. Once the installation is complete, you will be prompted to restart your computer. Remove the USB drive and press "Enter" to reboot.
11. After rebooting, log in with the username and password you created during the installation process.
12. Update your system by opening a terminal and running the following commands:
    ```bash
    sudo apt update
    sudo apt upgrade -y
    ```
13. [recommended] install Yakuake terminal emulator for easier terminal management, and standardize instructions in documentation:
    ```bash
    sudo apt install -y yakuake
    ```
    you can launch yakuake by pressing F12 key, or by searching for "Yakuake" in the application menu.
    open yakuake > profile settings > scrolling > enable unlimited scrollback > save.
14. Install any additional drivers if necessary by going to "Software & Updates" > "Additional Drivers" tab and selecting the appropriate drivers for your hardware.
15. Restart your computer if prompted to complete the driver installation.
16. Your Ubuntu 20.04 installation is now complete and ready for use! You can now proceed with installing any additional software or configurations as needed.
17. Do not upgrade to a newer Ubuntu version (22.04 or later) as it is not yet supported for drone leaf software stack.
18. Change Region settings to your location:
    - Open "Settings" > "Region & Language"
    - Set "Formats" to "United Kingdom" for standardized date/time/number formats used in documentation.
19. Rename hostname to "SITL" for easier reference in documentation:
    - Open a terminal and run the following command:
    ```bash
    sudo hostnamectl set-hostname SITL
    ```
20. Add 16GB of swap file to improve system stability during heavy compilation tasks:
    ```bash
    free -h
    sudo fallocate -l 16G /swapfile
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
    sudo swapon /swapfile
    echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
    ```
21. Do reboot after all installation and setup steps are completed.
---
Important summary:
- Ubuntu 20.04 LTS is installed
- You have internet access (ethernet or wifi)
- minimum 128 GB free disk space (256 GB recommended)
- apt is updated and upgraded
- Yakuake terminal emulator is installed
- user name is ubuntu, as "ubuntu" might be refferred to in documentation


### personalize your installation
- Go to "Settings" > "Appearance" and choose your preferred theme (light/dark) and accent color.
- Set your desktop background by right-clicking on the desktop and selecting "Change Background."
- Customize your dock settings by going to "Settings" > "Dock" and adjusting the size, position, and behavior of the dock.
- Install your favorite internet browser (e.g., Firefox, and any chrome-based browsers).
- link your online accounts by going to "Settings" > "Online Accounts" and adding your accounts (e.g., Google, Nextcloud, etc.) for easy access to emails, calendars, and files.
- do not enable livepatch or any other kernel auto-update tools, as they may cause instability with drone leaf software stack.

### Post-Installation Steps: basic environment setup
1. Install following apt packages: [use yakuake terminal for all commands below. Rename tab name from "shell" to "env_setup" for easier reference for debugging in documentation]
    ```bash
    sudo apt install -y build-essential libdbus-glib-1-dev libgirepository1.0-dev git curl wget cmake unzip pkg-config libssl-dev libjpeg-dev libpng-dev libtiff-dev libusb-1.0-0-dev python3-pip jq
    ```
2. Preparation for github:
    - Open firefox and go to github.com
    - Sign in to your github account
    - make sure you have access to hear-cli repository:
    https://github.com/DroneLeaf/HEAR_CLI
    - navigate to Settings > Developer settings > Personal access tokens > Tokens(classic)
    - Generate new token:
        - Token name: DroneLeaf Token
        - Description: Token for DroneLeaf hear-cli repository access
        - Resource owner: if you are part of DroneLeaf organization, select DroneLeaf, otherwise select your personal account
        - Expiration: 366 days
        - Repository access: all repositories
        - Permissions > Repositories:
            - set "Contents" to "Read and write"
        - Click "Generate token" at the bottom of the page  
        - the page will reload and show you the generated token, keep the page open for now (you will not be able to see the token again), we will copy it in the next steps. You will need it three times!
    - run following command in terminal to configure git with your github account (replace with your actual github email and name):
    ```bash
    git config --global user.email "your_email@example.com"
    git config --global user.name "Your Name"
    ```
    - run following command in terminal to cache your github token for authentication (replace with your actual token):
    git config --global credential.helper 'cache --timeout=3600'
    git credential approve <<EOF
    protocol=https
    host=github.com
    username=your_github_username
    password=your_personal_access_token
    EOF
    ```
    Obviously, replace `your_github_username` and `your_personal_access_token` with your actual GitHub username (found in your GitHub account settings) and the token you just generated.

3. Follow the instructions in the [Hear_CLI readme](https://github.com/DroneLeaf/HEAR_CLI) to install hear-cli tool and clone necessary repositories.(run in yakuake terminal "env_setup" tab)

4. Install required docker containers and install the SITL full system:
    - Open a new yakuake tab, rename it to "hear_docker_clone" run following command:
    ```bash
    hear-cli local_machine run_program --p hear_docker_clone    
    ```
    - input your github username and personal access token when prompted.
    
    - run following command(The output of the command is saved to docker_sitl_full_system.log): 
    ```bash
        hear-cli local_machine run_program --p hear_docker_sitl_full_system_install | tee docker_sitl_full_system.log
    ```
  
5. Reboot your system to ensure all changes take effect properly:
    ```bash
    sudo reboot
    ```

6. Continue with the installation:
    - Open a new yakuake tab, rename it to "hear_docker_setup": 
    - run following command, (the output of the command is saved to install_system_dependencies_sitl.log):
    ```bash
    hear-cli local_machine run_program --p install_system_dependencies_sitl | tee install_system_dependencies_sitl.log
    ```
    - reboot your system again:
    ```bash
    sudo reboot
    ```
7. After rebooting, open a new yakuake tab, rename it to "hear_docker_cleanup":
    - run following command (the output is saved to the log files mentioned after the tee command respectively):
    ```bash
    hear-cli local_machine run_program --p docker_install | tee docker_install.log
    hear-cli local_machine run_program --p node_install | tee node_install.log
    hear-cli local_machine run_program --p configure_software_setup_autostart_sitl | tee configure_software_setup_autostart_sitl.log
    ```
    - reboot your system again:
    ```bash
    sudo reboot
    ```

8. To continue, request env certificate from DroneLeaf support team [Ahmed Hashem](ahmed.hashim@droneleaf.io). Download the file in home as "env.zip", then run following command in a new yakuake tab renamed to "hear_docker_env_setup":
    ```bash
    [ -f "./env.zip" ] && unzip -o "./env.zip" -d ./env || echo "env.zip not found";
    mkdir -p "$HOME/HEAR_CLI/scripts/programs/init_ecr_pull_profile" "$HOME/HEAR_CLI/scripts/programs/init_sync_profile";
    if [ -f ./env/init_ecr_pull_profile/env ]; then cp -v ./env/init_ecr_pull_profile/env "$HOME/HEAR_CLI/scripts/programs/init_ecr_pull_profile/env"; else echo "Warning: ./env/init_ecr_pull_profile/env not found"; fi;
    if [ -f ./env/init_sync_profile/env ]; then cp -v ./env/init_sync_profile/env "$HOME/HEAR_CLI/scripts/programs/init_sync_profile/env"; else echo "Warning: ./env/init_sync_profile/env not found"; fi;
    ```
9.  Initialize ECR pull profile:
    ```bash
    hear-cli local_machine run_program --p init_ecr_pull_profile
    ```
10. Initialize sync profile:
    ```bash
    hear-cli local_machine run_program --p init_sync_profile
    ```
11. Initialize data lifecycle preparation:
    ```bash
    hear-cli local_machine run_program --p data_lifecycle_prepare
    ```
12. - Reboot your system to ensure all changes take effect properly:
    ```bash
    sudo reboot
    ```

13. Initialize controller dashboard preparation:
    ```bash
    hear-cli local_machine run_program --p controller_dashboard_prepare
    hear-cli local_machine run_program --p software_stack_clone 
    #Note: Choose the dev-sitl branch if you want to install a development environment, otherwise choose main branch [as of Oct 2025, main is way behind. Use dev for stable release, or dev-sitl for development].
    hear-cli local_machine run_program --p set_fc_configs
    ```
14. Optional: Install petal app manager and prepare sitl
    ```bash
    hear-cli local_machine run_program --p petal_app_manager_prepare_sitl
    ```
15. reboot your system to ensure all changes take effect properly:
    ```bash
    sudo reboot
    ```
16. Check points for successful installation:
    - Open yakuake terminal and run:
        ```bash
        docker ps
        ```
        You should see multiple running containers related to drone leaf software stack.
    - check mavlink-router.serivce status:
        ```bash
        systemctl status mavlink-router.service
        ```
        You should see that the service is active and running.
17. Building PX4-Autopilot:
    - Open a new yakuake tab, rename it to "px4_build":
    - install following python packages:
    ```bash
    pip3 install kconfiglib
    pip3 install --user jsonschema
    pip3 install --user pyros-genmsg
    pip3 install --user jinja2
    ```
    - run following commands:
    ```bash
    cd ~/software-stack/PX4-Autopilot
    make px4_sitl gazebo-classic
    ```
        - options for different build options:
            - make px4_sitl gazebo-classic_dfl # for DFL
            - HEADLESS=1 make px4_sitl gazebo-classic # for headless build [lower resource usage]

18. Running LeafQGC [AKA: LeafMC]:
    - Open a new yakuake tab, rename it to "leafQGC":
    - run following commands:
    ```bash
    cd ~ # navigate to home directory
    git clone -b dev-sitl https://github.com/DroneLeaf/LeafMC.git # clone LeafMC repo, use dev-sitl branch for latest development version
    cd LeafMC
    sudo ./tools/install-dependencies-debian.sh # install LeafMC build dependencies
    ```
    - Make sure Qt 5.15.2 is installed on your system. You can check the installed version by running:
    ```bash
    qmake --version
    ```
    - If Qt 5.15.2 is not installed, install it using hear-cli:
    ```bash
    hear-cli local_machine run_program --p install_qt
    ```
    - install Qt Creator:
    ```bash
    sudo apt-get install qtcreator
    ```
    - Open Qt Creator, open LeafMC/QGroundControl.pro file.
    - Configure the kits to use Qt 5.15.2. by going to "Projects" in the left pane, selecting Qt 5.15.2 with GCC 64.
    - In environment, edit Build Environment so that under "PATH", it starts with:
    ```bash
    /home/yo/Qt/5.15.2/gcc_64/bin:/usr/bin:/home/yo/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin
    ```
    - You can now build and run LeafQGC from Qt Creator.

19. Syncing Database from cloud, and registering the current SITL as a drone: [Note: This tutorial is from HEAR_wiki/Guide/HEAR Software/Operation/SITL
/Readme.md]
    - Make sure you are registered in `fly.droneleaf.io` and that your drone is bound to a license.
    
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

### Compiling HEAR_FC
Navigate to `~/software-stack/HEAR_FC` and compile the FC with SITL target:

```bash
cd ~/software-stack/HEAR_FC
catkin_make -DCMAKE_BUILD_TYPE=Debug -DHEAR_TARGET=SITL
source devel/setup.bash
```

In case you needed to recompile clean:
```bash
cd ~/software-stack/HEAR_FC
rm -rf devel/ build/
```

```bash
source devel/setup.bash
```

Launch the Flight Controller:

```bash
roslaunch flight_controller px4_flight_mavlink_opti_onboard_mission.launch
```

Debugging HearFC:
 - Open a new window in VS Code.
 - Open directory: `~/software-stack/HEAR_FC/src/HEAR_FC`
 - Go to Run and Debug tab.
 - Select "ROS: Launch Dbg px4_flight_mavlink_opti_onboard_mission.launch"
    ```json{
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
    }```
- Click on the green play button to start debugging.

### DynamoDB debugging and correcting incorrect configurations
  - Copy `robot_instance_id` , `robot_type_id` and `organization_id` from http://0.0.0.0:8080/table/config-robot_instances?tabActive=search
  - Copy `address` from http://0.0.0.0:8080/table/config-profile?tabActive=search of `"SITL"`
  - Edit: http://0.0.0.0:8080/table/config-robot_instance_profile_assignment?tabActive=search:
  - paste copied address as profile_id
  - paste copied robot_instance_id as robot_instance_id
  - Edit: http://0.0.0.0:8080/table/config-robot_type_allocation_assignment?tabActive=search
  - paste copied `robot_type_id` as robot_type_id
  - paste copied `organization_id` as organization_id
  - make sure that you have correct array structure in: 'http://0.0.0.0:8080/table/config-control_saturation?tabActive=search'
      - Check if the table exists, if not: fill:
      ```json
      {
      "ori_x_bias": 0.013594605028629303,
      "ori_z_bias": 0.13921412825584412,
      "address": "",
      "robot_type_id": "",
      "organization_id": "",
      "ori_y_bias": 0.022619375959038734,
      "id": "bcda0a97-50d1-40e2-b94c-1d3b10aa68f8",
      "robot_instance_id": `robot_instance_id`
      }
      ```
  - save document after edits, and restart the HEAR_FC launch file.

Knowledge base:
- expected file structure after software-stack clone:
    ```bash
    ~/software-stack/
    ├── PX4-Autopilot
    ├── LeafMC
    ├── MAVSDK
    ├── ...
    ```
    - Note: Do not change contents of software-stack directory manually, as it may interfere with hear-cli management of the repositories. For development on any of the repositories, create a separate clone outside of software-stack directory. [home directory is recommended location for development clones
    - Example:
    ```bash
    ~/PX4-Autopilot/
    ~/LeafMC/
    ```]


# ## leafQGC Deployment steps:
# note: leafQGC is a fork of QGroundControl with customizations for DroneLeaf's leafFC. It is based on QGroundControl version 4.4.2
# - clone repo ['leafMC']
# - install dependencies from /tools/*
# - install Qt 5.15.2
# - install Qt Creator 4.13.2
# - open CMakelists.txt in Qt Creator
# - customize the kits to use Qt 5.15.2
#     yo@yo-IdeaPad-Slim-3-15IRH8:~$ qmake --version
#     QMake version 3.1
#     Using Qt version 5.15.2 in /home/yo/Qt/5.15.2/gcc_64/lib
# - then, in Qt Version tab, remove all other Qt versions except 5.15.2. If 5.15.2 is not listed, add it using "Add" button and navigating to /home/yo/Qt/5.15.2/gcc_64/bin/qmake.
# - click on apply
# - update qt in kits to use 5.15.2
# - click on OK
# - click on configure project
# - if the error message is "License check failed, Giving up.", then:
#     - go to Tools > Options > License
#     - click on "Add" button
#     - select "I have a license key file"
#     - navigate to /home/yo/Qt/Tools/QtCreator/licenses/qt-creator-license.xml
#     - click on OK
#     - restart Qt Creator
#     - reopen QGroundControl.pro
# - now, you should be able to build the project without license errors.


# - build and run
