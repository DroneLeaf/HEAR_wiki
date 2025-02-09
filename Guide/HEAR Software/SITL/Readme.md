# Overview

This document describes how the SITL is run with PX4 gazebo environment.


## DroneLeaf developers

### Your first SITL run!

As you have your machine ready for development and testing, let's run the SITL setup to play around the software stack.

#### 1 - Mavlink Router


```bash
sudo mkdir -p /etc/mavlink-router/
```

```bash
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
Port = 10000
' | sudo tee /etc/mavlink-router/main.conf
```

First of all, before launching Leaf QGC, we need to run the following command.
```bash
mavlink-routerd 0.0.0.0:14550
```
After successful execution, please make sure your settings are the same as the output below.
```
Opened UDP Server [4]server: 0.0.0.0:10000
Opened UDP Client [5]client: 0.0.0.0:11000
Opened UDP Server [7]CLI: 0.0.0.0:14550
Opened TCP Server [8] [::]:5760
```
If everything is correct, you can continue to the next step.


#### 2 - PX4-Autopilot

```bash
cd PX4-Autopilot
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

#### 3 - Leaf QGroundControl

Download QGC
https://droneleafworkspace.slack.com/files/U080N4HR8HG/F086D7BCQQ7/leafmc.appimage


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

#### 4 - HEAR_Configurations
As you have the PX4 SITL running and connected to QGroundControl, you need to setup the configurations before runing the HEAR_FC. There are a couple of settings you have to make sure that setup correctly.

- Systems/Pixhawk
- UAV_instances

Head to ~/HEAR_Configurations/Systems/Pixhawk/ and edit the general.json file to match the following:

```json
{
    "pixhawk_ip": "127.0.0.1",
    "pixhawk_udp_port": 10000,
    "local_ethernet_interface_ip": "127.0.0.1",
    "local_udp_port": 14540,
    "serial_port_addr":"/dev/ttyS0",
    "serial_baud_rate":921600
}
```
You need to set the "pixhawk_ip" and "local_ethernet_interface_ip" to your local IP.

After setting up the Pixhawk configuration, you need to check the general.json file in ~/HEAR_Configurations/UAV_instances/ and edit the "default_uav_instance_name" according to the drone instance that you want to simulate.

```json
{
    "default_uav_instance_name": "X500_TF40d_Test",
    "Description": "Define default_uav_instance_name to enforce uav instance name on the current machine"
}
```

#### 5 - HEAR_FC
As you have the PX4 SITL running and connected to QGroundControl, you can now run the HEAR_FC to simulate the drone's flight controller.

- Run
    ```bash
    cd ~/HEAR_FC
    source devel/setup.bash
    roslaunch flight_controller px4_flight_mavlink_opti.launch
    ```
Finally, you should see "PX4 Ready to Fly" message on the QGC and you are good to go!
