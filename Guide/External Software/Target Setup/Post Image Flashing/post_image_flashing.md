

**HINT: the new rpi image has static ip of 10.0.0.250**

```
hear-cli local_machine run_program --p hear_cli_pull


hear-cli target copy_run_program --p system_extend_root_partition 
hear-cli target copy_run_program --p system_extend_root_partition 

hear-cli target copy_run_program --p system_resize2fs 

hear-cli target copy_run_program --p set_static_eth0_ip_specific_interface

hear-cli target copy_run_program --p set_static_ip_wifi_specific_interface

hear-cli target copy_run_program --p data_lifecycle_prepare 

hear-cli target copy_run_program --p controller_dashboard_prepare 

ear-cli target copy_run_program --p configure_software_setup_autostart_arm 

hear-cli target copy_run_program --p set_image_patch_number_01

```


- License the drone and sync data