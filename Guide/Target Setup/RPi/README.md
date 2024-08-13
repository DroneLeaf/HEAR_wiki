

# Image Preparaion

### Burn Fresh Image on target
1- Download Raspberry Pi Imager App 
```
sudo snap install rpi-imager
```



2- Connect Raspberry pi to local machine if it has `emmc memory`, or insert the `SD card` if availiable.

> If you use `emmc` version , turn-on boot switch,
convert it to usb memory on your device via this commands, so you can view emmc storage as a usb memory.
```bash

# install usbboot
###### only doing once.
sudo apt install git libusb-1.0-0-dev pkg-config build-essential
git clone --recurse-submodules --shallow-submodules --depth=1 https://github.com/raspberrypi/usbboot
cd usbboot
make

# Run usbboot
sudo ./rpiboot
```

3- Open Raspberry Pi Imager App.

4- From `Operating System` Option Select `"NO FILTERING"`

5- From `Raspberry pi Device` Option Select `"Other General-Purpose OS ===> Ubuntu ===> Ubuntu Server 20.04.5 LTS (64-bit)"`

6- From `Storage` Option Select your pi storage you need to flash.

7-Press `Next`

8-Press `Edit Settings`
- Press `GENERAL` Tab
  - Activate Username and pasword option ===> type user name "pi" ===> type password "raspberry"

  - Activate Configure Wireledd LAN option ===> type your wifi SSID and password. > so you can connect to raspberry pi after flashing via wifi.
- Press `SERVICES` Tab
  - Activate Enable SSH ===> choose `use password authentication`

- Press `Save`
- Press `yes`
- Press `yes`

> Then wait untl flashing is completed successfully. ☕🍪



### Full System Installation

> this step depends on `HEAR_CLI` https://github.com/ahmed-hashim-pro/HEAR_CLI

1-Clone HEAR_CLI
```
git clone https://github.com/ahmed-hashim-pro/HEAR_CLI
```

2- Follow hear-cli installation instructions in HEAR_CLI repo  https://github.com/ahmed-hashim-pro/HEAR_CLI?tab=readme-ov-file#install-hear-cli


3- Power-ON Target.

4- Detect the ip of the target

5- Add New instance and fleet with propper configurations in `~/HEAR_Configurations` folder, so we can use the new fleet with hear-cli  

6- Clone HEAR_Docker on target via `hear-cli`

```bash
hear-cli fleet copy_run_program
```
Choose the proper program EX:`hear_docker_clone` , enter your fleet name thet you created before ,wait until the program is finish executing.


7- Start full system installation process from hear-cli

```bash
hear-cli fleet copy_run_program
```

Choose the proper program EX:`hear_docker_rpi_full_system_install` , enter your fleet name thet you created before ,wait until full system installation is finished.



### Other Preparations

1- Clone `HEAR_Configurations on target` via `hear-cli` program

```bash
hear-cli fleet copy_run_program
```

Choose the program `clone_hear_configurations`, enter your fleet name thet you created before ,wait until the program is finish executing.


2- Copy `~/HEAR_FC/src/HEAR_FC/Flight_controller/launch` to target via `hear-cli` command

```bash
hear-cli fleet copy --local-dir="~/HEAR_FC/src/HEAR_FC/Flight_controller/launch" --remote-dir="HEAR_FC/src/HEAR_FC/Flight_controller/launch"
```
> HINT: `--remote-dir` is start from home directory in target machine.


3- Copy `~/HEAR_FC/devel` to target via `hear-cli` command

```bash
hear-cli fleet copy --local-dir="~/HEAR_FC/devel" --remote-dir="HEAR_FC/devel"
```
> HINT: `--remote-dir` is start from home directory in target machine.


4- Clean target from unnecessary files

```bash
hear-cli fleet copy_run_program
```

Choose the program `clean_target_for_deployment`, enter your fleet name thet you created before ,wait until the program is finish executing.




### Deployment Steps

1- Set Static ip on target


```bash
hear-cli fleet copy_run_program
```

Choose the program `set_static_ip`, enter your fleet name thet you created before ,wait until the program is finish executing.