

# Image Preparaion

### Burn Fresh Image on target
1- Open PI Imager

2- Attach Raspberry pi to local machine

3- Choose Desired os 

4- Start Burn process

5- Edit Configuration to add wifi and activate ssh on fresh Image




### Full System Installation

> this step depends on `HEAR_CLI` https://github.com/ahmed-hashim-pro/HEAR_CLI

1- Power-ON Target.

2- Detect the ip of the target

3- Add New instance and fleet with propper configurations in `~/HEAR_Configurations` folder, so we can use thw new fleet with hear-cli  

4- Clone HEAR_Docker on target via `hear-cli`

```bash
hear-cli fleet copy_run_program
```
Choose the proper program EX:`hear_docker_clone` , enter your fleet name thet you created before ,wait until the program is finish executing.


4- Start full system installation process from hear-cli

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


2- Copy `~/HEAR_FC/devel` to target via `hear-cli` command

```bash
hear-cli fleet copy --local-dir="~/HEAR_FC/devel" --remote-dir="HEAR_FC/devel"
```
> HINT: `--remote-dir` is start from home directory in target machine.


3- Clean target from unnecessary files

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