# Moving between SITL (PX4 Autopilot) and a physical Pixhawk

This short runbook explains the minimal changes required when switching your workstation between SITL and bench (physical Pixhawk) operation. Style follows the other SITL guides in this repo.

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

4. Update the mavlink-router config to use the Pixhawk endpoint.
   - Replace the default/commented client (0.0.0.0:11000) with the Pixhawk client:
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
   - Restart the service:
     sudo systemctl restart mavlink-router.service

Notes
- The config path used above is /etc/mavlink-router/main.conf. See the project README for more detail: [Guide/External Software/MAVLink/mavlink-router/README.md](Guide/External Software/MAVLink/mavlink-router/README.md).

## DynamoDB — switch machine profile (SITL ↔ Bench)
1. Open the DynamoDB manager UI:
   - Config profiles: http://localhost:8080/table/config-profile?tabActive=search
   - Robot-instance assignments: http://localhost:8080/table/config-robot_instance_profile_assignment?tabActive=search

2. Steps:
   - On the config-profile page, locate the row for "SITL" or "Bench-Testing" and copy the profile's id / IP field you need.
   - Open config-robot_instance_profile_assignment, find the record for your machine, and update the `profile_id` with the copied value. Save.

3. For more background on DynamoDB sync and HEAR_FC setup, see:
   [Guide/HEAR Software/Operation/SITL/DynamoDB-and-hearfc-debugging.md](Guide/HEAR Software/Operation/SITL/DynamoDB-and-hearfc-debugging.md)

## Apply changes / restart
- If you can reboot, do so:
  sudo reboot
- Otherwise restart the services manually:
  sudo systemctl restart mavlink-router.service
  sudo systemctl restart leafFC.service
- Restart PX4 autopilot (physical Pixhawk: power-cycle the device; SITL: restart your px4_sitl process).

## Troubleshooting
- If you cannot ping the Pixhawk, re-check cabling, interface IP, and subnet mask.
- If mavlink-router fails to start, inspect logs:
  sudo journalctl -u mavlink-router.service -n 200 --no-pager
