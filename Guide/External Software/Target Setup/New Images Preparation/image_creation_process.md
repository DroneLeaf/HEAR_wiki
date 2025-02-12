# Install All dependieces before taking snapshot

hear-cli target copy_run_program --p hear_docker_clone 

- For ORIN.

hear-cli target copy_run_program --p hear_docker_orin_full_system_install 

- For RPI.

hear-cli target copy_run_program --p hear_docker_rpi_full_system_install 

hear-cli target copy_run_program --p activate_network_timezone 

hear-cli target copy_run_program --p clean_target_for_deployment 

hear-cli target copy_run_program --p systemd_unattended_upgrades_disable 

hear-cli target copy_run_program --p systemd_networkd_wait_online_disable 

hear-cli target copy_run_program --p system_resize2fs 

hear-cli target copy_run_program --p hear_cli_install 

hear-cli target copy_run_program --p install_system_dependencies_arm

hear-cli target copy_run_program --p docker_install 

hear-cli target copy_run_program --p node_install 

hear-cli target copy_run_program --p save_image_version_number 

hear-cli target copy_run_program --p save_image_patch_number 

hear-cli target copy_run_program --p clean_target_for_deployment 

hear-cli target copy_run_program --p docker_install 

hear-cli target copy_run_program --p node_install 

hear-cli target copy_run_program --p set_static_eth0_ip_specific_interface 

hear-cli target copy_run_program --p install_system_dependencies_arm 

hear-cli target copy_run_program --p configure_software_setup_autostart_arm 


hear-cli target copy_run_program --p init_ecr_pull_profile 

hear-cli target copy_run_program --p init_sync_profile

hear-cli target copy_run_program --p data_lifecycle_prepare 

hear-cli target copy_run_program --p controller_dashboard_prepare 

hear-cli target copy_run_program --p set_fc_configs 

 