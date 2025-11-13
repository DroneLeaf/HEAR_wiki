# Moving between SITL (PX4 Autopilot) and a physical Pixhawk

The default configuration of mavlink router following SITL setup is to connect to the SITL instance of PX4 running on the same machine. To switch to a physical Pixhawk device, do the following steps:

## Overview
- Update MAVLink Router to talk to the Pixhawk IP/port.
- Update the machine profile (SITL ↔ Bench) in the local DynamoDB UI.
- Restart services (or reboot) so changes take effect.

## MAVLink Router — network and config changes
1. Physically connect the Pixhawk to the ethernet port of your machine.
2. Set a static IP on the workstation for the Pixhawk network:
   - Find your wired connection name:
     sudo nmcli connection show
   - Replace <CONN> with the connection name and run:
     sudo nmcli connection modify <CONN> ipv4.addresses 192.168.144.5/24 ipv4.method manual
     sudo nmcli connection up <CONN>
3. Verify connectivity (replace IP if different in your setup):
   ping -c 4 192.168.144.4

4. Change MAVLink Router config:
    4.1 **Temporary change:**
    Make sure any systemd MAVLink router has been stopped first:

      ```bash
      sudo systemctl stop mavlink-router.service
      ```

      Then run 

      ```bash
      mavlink-routerd 0.0.0.0:14540 0.0.0.0:14550
      ```
    To revert back, stop the above process (Ctrl+C) and restart the service:

      ```bash
      sudo systemctl start mavlink-router.service
      ``` 

    4.2 **Permanent change:**
    Replace the default/commented client (Normal: 0.0.0.0:11000, Server: 0.0.0.0:10000) with the Pixhawk client:
    ```bash
     sudo bash -c 'cat > /etc/mavlink-router/main.conf <<EOF
     [UdpEndpoint client]
     Mode = Normal
     Address = 192.168.144.4 # Pixhawk IP address: 192.168.144.4, SITL default: 0.0.0.0
     Port = 14540            # Pixhawk port: 14540, SITL default: 11000

     [UdpEndpoint server]
     Mode = Server
     Address = 0.0.0.0
     Port = 14550
     EOF'
     ```
   - Restart the service:
    ```bash
    sudo systemctl restart mavlink-router.service
    ```
    To revert back, replace the above config with the original/default config and restart the service.
    >Notes
    The config path used above is `/etc/mavlink-router/main.conf`. See the project README for more detail: [Guide/External Software/MAVLink/mavlink-router/README.md](Guide/External%20Software/MAVLink/mavlink-router/README.md).

## Choose the Bench-Testing profile instead of the SITL profile from the client web app
> It is reported that the client web app does not have an option to switch between SITL and Bench profiles. See [Known Issues](./known_issues.md#switching-between-sitl-and-bench-profiles-no-option-in-the-web-ui) for a workaround.



## Troubleshooting
- If you cannot ping the Pixhawk, re-check cabling, interface IP, and subnet mask.
