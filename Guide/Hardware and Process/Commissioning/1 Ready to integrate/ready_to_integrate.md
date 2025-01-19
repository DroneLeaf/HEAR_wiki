# Target Preparation For `Ready_To_Integrate` Process

## Burn image to target
Find images in: https://droneleaf.sharepoint.com/:f:/s/technical/Eg2LOo9V0ANArVuQ_eN8DWIBshhVyvY0ty1XDy2FspXfbg?e=CflYt8
Check release notes in relevant version file found above.

### Download the latest version of desired target

the image folder will contain `*.zip` file and `md5sum.txt` file.

`md5sum.txt` file contain generated `md5sum` signature of current image file

after uncompress the *.zip file , make sure to Run
```bash
md5sum *.img
```
file signature will be extracted and then you have to compare with the value in the `md5sum.txt` file.
### Use Balena Etcher For Burning the Image
Use Balena Etcher to burn the proper target image inside the new target (`ORIN`,`RPI`, etc)

download latest version from here : https://github.com/balena-io/etcher/releases/

and then install with this command`sudo apt install ./balena-etcher_******_amd64.deb`
> make sure to replace stars with file version

Run Balena Etcher and flash the image from file to the boot-up memory by doing the following steps:

- Connect the SSD drive or EMMC memory to your machine:
- Open balena etcher.
- Press `flash from file` and select the downloaded image file.
- press `select` target and then choose the memory you already attached.
- press `flash` , then wait until it is finished.

### Note on additional steps for RPi with EMMC
Install:

```bash

# install usbboot
###### only doing once.
cd
sudo apt install git libusb-1.0-0-dev pkg-config build-essential
git clone --recurse-submodules --shallow-submodules --depth=1 https://github.com/raspberrypi/usbboot
cd usbboot
make
```

> If you use `emmc` version RPi, turn-on boot switch (if you are using Pixhawkv6x carrier board with CM4 then toggle the switch to EMMC position). This will convert the RPi to usb memory on your device via this commands, so you can view emmc storage as a usb memory on your OS.

To load the RPi EMMC memory use:
```bash
# Run usbboot from the path of the cloned repo above
cd ~/usbboot
sudo ./rpiboot -l
```
## Extend target memory size
Add unallocated size of the storage to system partition .
- Install `GParted`

```bash
sudo apt install gparted
```

- Open `GParted`
- Select your memory disk from the list at top-right app corner.
- You will find multiple partitions, make sure to select the right one:
  -   it is always has the largest size
  -   file system is `ext4`
- Right click on the selected partition from previous step.
- Choose `Resize/Move` and a Resize/Move window will be displayed.
- Click on the right-hand side of the partition and drag it as far to the right as possible. 
- Click `Resize/Move` button to queue the operation.
- From App Menu Choose `Edit` | `Apply All Operations` menu option to apply the queued operations, to disk.
- Click on `Apply` to apply operations to disk.
- If any alert is appeared telling you to fix disk , just click `Fix`.
- Click on `Close` to close the apply operations to disk window.

Now You are done.
Remove the target disk from your local machine and connect it to the target machine
> If you use `emmc` version RPi, turn-off boot switch (if you are using Pixhawkv6x carrier board with CM4 then toggle the switch to RPI position). 

## Post image burning installations and configurations

- Connect the target to LAN through the device ethernet port.
- Set static IP.


   - for wifi interface
     ```bash
     hear-cli target copy_run_program --p set_static_ip_wifi_specific_interface
     ```

   - for ethernet interface
     ```bash
     hear-cli target copy_run_program --p set_static_eth0_ip_specific_interface
     ```


- Add instance data to HEAR_Configurations with current target connection data.

- **deprecated** Clone HEAR_Configurations 
```bash
hear-cli instance copy_run_program --p clone_hear_configurations
```

- **deprecated** HEAR_Msgs clone

```bash
hear-cli instance copy_run_program --p hear_msgs_clone
```
- **deprecated** HEAR_Msgs build

```bash
hear-cli instance copy_run_program --p hear_msgs_build
```
- **deprecated** HEAR_FC clone and catkin_clean

```bash
hear-cli instance copy_run_program --p hear_fc_clone_catkin_clean
```
- **deprecated** Ros IP Changer

```bash
hear-cli instance copy_run_program --p ros_ip_changer
```

- Change Network TimeZone

```bash
hear-cli instance copy_run_program --p activate_network_timezone
```
- Clean Target For Deployment

```bash
hear-cli instance copy_run_program --p clean_target_for_deployment
```

```bash
hear-cli instance copy_run_program --p systemd_unattended_upgrades_disable
```


```bash
hear-cli instance copy_run_program --p systemd_networkd_wait_online_disable
```


```bash
hear-cli instance copy_run_program --p system_resize2fs
```

```bash
hear-cli instance copy_run_program --p hear_cli_install
```



```bash
hear-cli instance copy_run_program --p install_system_dependencies
```

```bash
hear-cli instance copy_run_program --p configure_software_setup_autostart_rpi
```

```bash
hear-cli instance copy_run_program --p download_and_install_software_stack
```



```bash
hear-cli target copy_run_program --p save_image_version_number
```

```bash
hear-cli target copy_run_program --p save_image_patch_number
```



