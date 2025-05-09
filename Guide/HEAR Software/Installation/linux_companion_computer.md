# Installation Guide for DroneLeaf's Flight Controller Software (LeafFC) on Nvidia Jetson Orin
📝 **🔴Important Note Specific for SalukiV3 Collaboration Context🔴**: The scripts enclosed in this submission are provided for *reference only*. Some commands will not work due to remote permission restrictions and security measures. The objective is to provide necessary awareness for package dependencies which shall be achievable with the current format.

## 🚀 NVIDIA JetPack and Linux Version
[NVIDIA JetPack 5.1.4](https://developer.nvidia.com/embedded/jetson-linux-r3560) is a production quality release. It includes Jetson Linux 35.6.0 BSP with Linux Kernel 5.10, an Ubuntu 20.04 based root file system, a UEFI based bootloader, and OP-TEE as Trusted Execution Environment. 

The hardware we use is the Jetson Orin Nano.

📝 **Note**: We specifically support Ubuntu 20.04 as LeafFC depends on some ROS 1 packages.

## 📂 Script Organization Note
All installation and setup scripts are organized under the `functions/` directory to maintain modularity and reusability. Scripts are grouped by feature or component.

For example:
```bash
functions/HEAR_Docker/hear_docker_clone.sh
```

To run a script manually, you can execute it from the root of the repository:

```bash
bash functions/HEAR_Docker/hear_docker_clone.sh
```

## 📜 A script for package installation

```bash
# Installing HEAR Docker
functions/HEAR_Docker/hear_docker_clone.sh main
functions/packages_installer.sh 

# Install target specific dependencies
target=ORIN
cd ~/HEAR_Docker
sudo chmod -R +x src/targets/$target'_UBUNTU20'
cd src/targets/$target'_UBUNTU20'
./full_system_installation.sh hear_docker_orin_full_system_install

# Function to set the timezone
set_timezone() {
  # echo "Available timezones:"
  # timedatectl list-timezones

  # echo
  # read -p "Enter the desired timezone (e.g., America/New_York): " timezone

  if sudo timedatectl set-timezone "$timezone"; then
    echo "Timezone set to $timezone"
  else
    echo "Failed to set timezone. Please ensure the timezone is valid."
    exit 1
  fi
}


synchronize_time() {
  echo "Synchronizing time with NTP..."
  
  if sudo timedatectl set-ntp true; then
    echo "NTP enabled. Time synchronization started."
  else
    echo "Failed to enable NTP. Please ensure you have network access."
    exit 1
  fi
}

set_timezone
synchronize_time

# Disable unattended upgrades
sudo functions/systemd/systemd-unattended-upgrades-disable.sh
sudo functions/systemd/systemd-networkd-wait-online-disable.sh

# Fix os partition size if needed
sudo functions/system/resize_os_partition.sh

###################################
### Install System Dependencies ###
###################################
TARGET_BUILD=ORIN
LIB_ARCH="linux-x64"
if [[ $TARGET_BUILD == "RPI" || $TARGET_BUILD == "ORIN" ]]; then
    LIB_ARCH="linux-arm64"
fi

# start sudo 
functions/sudo_init.sh $targetPass

for LIB_NAME in "${LIBRARIES[@]}"; do
    echo "Checking for $LIB_NAME..."
    
    # Execute the cmake command and capture the output
    OUTPUT=$(cmake --find-package -DNAME="$LIB_NAME" -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=EXIST 2>&1)

    # Check if the library is found
    if [[ $OUTPUT == *"$LIB_NAME found."* ]]; then
        echo "$LIB_NAME found."
        # Proceed with any common logic for found libraries if needed
    else
        echo "$LIB_NAME not found. Installing..."

        # Perform different logic based on the library name only if not found
        case $LIB_NAME in
            "RapidJSON")
                  git clone --recursive https://github.com/Tencent/rapidjson.git
                  cd rapidjson
                  mkdir build
                  cd build
                  cmake .. -DRAPIDJSON_BUILD_TESTS=OFF
                  make
                  sudo make install

                  cd ../..
                  rm -rf rapidjson

                ;;
            
            "cpr")
                sudo apt-get install libcurl4-openssl-dev libssl-dev -y
                git clone --recursive https://github.com/libcpr/cpr.git
                cd cpr
                mkdir build
                cd build
                cmake .. -DCPR_USE_SYSTEM_CURL=ON -DBUILD_SHARED_LIBS=OFF
                cmake --build . --parallel
                sudo cmake --install .

                cd ../..
                rm -rf cpr

                ;;

            "cpp-terminal")
                git clone --recursive https://github.com/jupyter-xeus/cpp-terminal.git
                cd cpp-terminal
                mkdir build
                cd build
                cmake .. -DCPPTERMINAL_ENABLE_TESTING=OFF
                make
                sudo make install

                cd ../..
                rm -rf cpp-terminal

                ;;
            
            "onnxruntime")
                # libonnxruntime install
                mkdir /tmp/onnxInstall
                cd /tmp/onnxInstall
                wget -O onnx_archive.nupkg https://www.nuget.org/api/v2/package/Microsoft.ML.OnnxRuntime/1.15.1
                unzip onnx_archive.nupkg

                cd runtimes/$LIB_ARCH/native/
                ln -s libonnxruntime.so libonnxruntime.so.1.15.1
                sudo cp libonnxruntime.so /usr/local/lib/
                sudo cp libonnxruntime.so.1.15.1 /usr/local/lib/

                cd /tmp/onnxInstall
                sudo mkdir -p /usr/local/include/onnxruntime/
                sudo cp -r build/native/include/ /usr/local/include/onnxruntime/

                sudo mkdir -p /usr/local/share/cmake/onnxruntime/
                sudo touch /usr/local/share/cmake/onnxruntime/onnxruntimeVersion.cmake
                printf '\n
                # Custom cmake version file by jcarius \n
                \n
                set(PACKAGE_VERSION "1.15.1")\n
                \n
                # Check whether the requested PACKAGE_FIND_VERSION is compatible\n
                if("${PACKAGE_VERSION}" VERSION_LESS "${PACKAGE_FIND_VERSION}")\n
                  set(PACKAGE_VERSION_COMPATIBLE FALSE) \n
                else()\n
                  set(PACKAGE_VERSION_COMPATIBLE TRUE) \n
                  if("${PACKAGE_VERSION}" VERSION_EQUAL "${PACKAGE_FIND_VERSION}") \n
                    set(PACKAGE_VERSION_EXACT TRUE) \n
                  endif() \n
                endif()\n
                ' | sudo tee /usr/local/share/cmake/onnxruntime/onnxruntimeVersion.cmake

                sudo touch /usr/local/share/cmake/onnxruntime/onnxruntimeConfig.cmake
                printf ' \n
                # Custom cmake config file by jcarius to enable find_package(onnxruntime) without modifying LIBRARY_PATH and LD_LIBRARY_PATH \n
                # \n
                # This will define the following variables: \n
                #   onnxruntime_FOUND        -- True if the system has the onnxruntime library \n
                #   onnxruntime_INCLUDE_DIRS -- The include directories for onnxruntime \n
                #   onnxruntime_LIBRARIES    -- Libraries to link against \n
                #   onnxruntime_CXX_FLAGS    -- Additional (required) compiler flags \n
                \n
                include(FindPackageHandleStandardArgs) \n
                \n
                # Assume we are in <install-prefix>/share/cmake/onnxruntime/onnxruntimeConfig.cmake \n
                get_filename_component(CMAKE_CURRENT_LIST_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH) \n
                get_filename_component(onnxruntime_INSTALL_PREFIX "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE) \n
                \n
                set(onnxruntime_INCLUDE_DIRS ${onnxruntime_INSTALL_PREFIX}/include/onnxruntime/include) \n
                set(onnxruntime_LIBRARIES onnxruntime) \n
                set(onnxruntime_CXX_FLAGS "") # no flags needed \n
                \n
                \n
                find_library(onnxruntime_LIBRARY onnxruntime \n
                    PATHS "${onnxruntime_INSTALL_PREFIX}/lib" \n
                ) \n
                \n
                add_library(onnxruntime SHARED IMPORTED) \n
                set_property(TARGET onnxruntime PROPERTY IMPORTED_LOCATION "${onnxruntime_LIBRARY}") \n
                set_property(TARGET onnxruntime PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${onnxruntime_INCLUDE_DIRS}") \n
                set_property(TARGET onnxruntime PROPERTY INTERFACE_COMPILE_OPTIONS "${onnxruntime_CXX_FLAGS}") \n
                \n
                find_package_handle_standard_args(onnxruntime DEFAULT_MSG onnxruntime_LIBRARY onnxruntime_INCLUDE_DIRS) \n
                ' | sudo tee /usr/local/share/cmake/onnxruntime/onnxruntimeConfig.cmake

                cd /tmp
                rm -rf onnxInstall

                ;;

            *)
                echo "No specific failure handling defined for $LIB_NAME."
                ;;
        esac
    fi
done

sudo apt-get install python3-bloom -y
sudo apt-get install fakeroot dpkg-dev debhelper -y
sudo apt-get install libxml2-utils -y

if ! command -v mavlink-routerd &> /dev/null
then
    echo "mavlink-routerd not found. Installing..."
    sudo pip3 install meson

    cd /tmp
    git clone --recursive https://github.com/mavlink-router/mavlink-router.git
    cd mavlink-router
    meson setup build .
    ninja -C build
    sudo ninja -C build install

    sudo mkdir -p /etc/mavlink-router/
    
    cd /tmp
    rm -rf mavlink-router
else
    echo "mavlink-routerd is already installed."
fi

sudo touch /etc/mavlink-router/main.conf

printf ' \n
    ## UART setup \n
    #[UartEndpoint pixhawk] \n
    #Device = /dev/ttyS0 \n
    #Baud = 52000  \n
    \n
    [UdpEndpoint client] \n
    Mode = Normal \n
    Address = 0.0.0.0 \n
    Port = 11000 \n
    \n
    [UdpEndpoint server] \n
    Mode = Server \n
    Address = 0.0.0.0 \n
    Port = 14550
    ' | sudo tee /etc/mavlink-router/main.conf

#################################

# Docker installation
functions/install_docker.sh

# Node 20 installation
functions/npm/install_node20.sh

# Update image and patch numbers
version_number=2
patch_number=1
python3 functions/save_version_patch.py --version_number $version_number --patch_number $patch_number



###################################
### Configure Software Autostart###
###################################

TARGET_RPI=OFF
TARGET_UBUNTU=OFF
TARGET_ORIN=ON
HEAR_TARGET=ORIN

username=$(whoami)
# setup mavlink-router service
sudo bash -c "cat > /etc/systemd/system/mavlink-router.service" <<EOL
[Unit]
Description=Mavlink Router service.

After=network-online.target

[Service]
ExecStart=mavlink-routerd 0.0.0.0:14540
RemainAfterExit=no
Restart=on-failure
RestartSec=2s

[Install]
WantedBy=multi-user.target
EOL

# setup roscore service
sudo bash -c "cat > /etc/systemd/system/roscore.service" <<EOL
[Unit]
Description=DroneLeaf FC launch service.

After=network-online.target

[Service]
User=${username}
ExecStart=/bin/bash -c "source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=http://localhost:11311; export ROS_HOME=/home/${username}/.ros; roscore"
RemainAfterExit=no
Restart=on-failure
RestartSec=2s

[Install]
WantedBy=multi-user.target
EOL

# setup roslauch service
sudo bash -c "cat > /etc/systemd/system/leafFC.service" <<EOL
[Unit]
Description=DroneLeaf FC launch service.
Requires=roscore.service
After=network-online.target roscore.service mavlink-router.service

[Service]
User=${username}
ExecStartPre=/bin/sleep 4
ExecStart=/bin/bash -c "source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=http://localhost:11311; export ROS_HOME=/home/${username}/.ros; source /home/${username}/HEAR_FC/devel/setup.bash ;roslaunch flight_controller px4_flight_mavlink_opti_onboard_mission.launch"
RemainAfterExit=no
Restart=on-failure
RestartSec=2s

[Install]
WantedBy=multi-user.target
EOL

sudo systemctl daemon-reload

sudo systemctl disable roscore.service
sudo systemctl disable mavlink-router.service
sudo systemctl disable leafFC.service

sudo systemctl stop roscore.service
sudo systemctl stop mavlink-router.service
sudo systemctl stop leafFC.service

sudo systemctl enable roscore.service
sudo systemctl enable mavlink-router.service
sudo systemctl enable leafFC.service

sudo systemctl start roscore.service
sudo systemctl start mavlink-router.service
sudo systemctl start leafFC.service
#################################

# Controller Dashboard related
python3 functions/aws/set_aws_profile.py



###################################
### Data lifecycle prepare      ###
###################################

# CALL file from functions folder
functions/data_lifecycle/OnboardExpressApi_pull_docker_compose.sh
functions/data_lifecycle/dynamodb-manager_pull_docker_compose.sh


cd ~/.droneleaf/OnBoardExpressApi
aws ecr get-login-password --region me-south-1 --profile ecrpull | docker login --username AWS --password-stdin 296257236984.dkr.ecr.me-south-1.amazonaws.com

docker-compose pull
docker-compose build --no-cache
cd $curr

cd  ~/.droneleaf/dynamodb-manager
# export HOST_IP=$(hostname -I | awk '{print $1}')
# echo "VUE_APP_PROXY_ENDPOINT=http://${target_ip}:4000/dynamodb" > .env
# echo "VUE_APP_PROXY_ENDPOINT=http://' + window.location.origin.split('//')[1].split(':')[0] + ':4000/dynamodb" > .env
aws ecr get-login-password --region me-south-1 --profile ecrpull | docker login --username AWS --password-stdin 296257236984.dkr.ecr.me-south-1.amazonaws.com

docker-compose pull

cd $curr

docker rm -f express_local
docker rm -f dynamodb_manager
docker rm -f dynamodb-local
docker rm -f proxy_server

# Check if the network exists
if ! docker network ls | grep -q "shared-network"; then
    docker network create shared-network
else
    echo "Network 'shared-network' already exists. Skipping creation."
fi

functions/data_lifecycle/dynamodb-manager_auto_start.sh
functions/data_lifecycle/OnboardExpressApi_auto_start.sh


sudo chmod -R 777 ~/.droneleaf/dynamodb


########################################
### Controller dashboard preparation ###
########################################
# CALL file from functions folder
functions/data_lifecycle/controller-dashboard_clone.sh

cd ~/.droneleaf/Controller_Dashboard
cd id_file_server
npm i
npm i -g webpack  -y
sudo chmod -R 777
sudo chmod +x compile_src.sh
npm run build
rm -rf ~/.droneleaf/Controller_Dashboard_Express_Api
mkdir -p ~/.droneleaf/Controller_Dashboard_Express_Api
cp -R dist/* ~/.droneleaf/Controller_Dashboard_Express_Api/
cp -R node_modules ~/.droneleaf/Controller_Dashboard_Express_Api/node_modules
cd ..
rm -rf id_file_server
cd $curr

functions/data_lifecycle/controller-dashboard_pull_docker_compose.sh

cd ~/.droneleaf/Controller_Dashboard
aws ecr get-login-password --region me-south-1 --profile ecrpull | docker login --username AWS --password-stdin 296257236984.dkr.ecr.me-south-1.amazonaws.com

docker-compose pull
cd $curr

sudo systemctl stop apache2.service
sudo systemctl disable apache2.service
docker stop file-server_app
docker rm file-server_app
docker stop machine_id_app
docker rm machine_id_app


docker rm controller_dashboard_app

username=$(whoami)

functions/data_lifecycle/Controller_Dashboard_auto_start.sh
functions/data_lifecycle/Controller_Dashboard_express_api_server_auto_start.sh
sudo functions/data_lifecycle/setup-systemctl-permissions.sh $username


sudo mkdir -p ~/.droneleaf
sudo chmod -R 777 ~/.droneleaf
########################################

functions/data_lifecycle/set_fc_configs.sh
```


---

© 2025 DroneLeaf AI Solutions LLC. All rights reserved.
