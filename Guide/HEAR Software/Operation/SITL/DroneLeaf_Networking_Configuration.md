<!-- file name: DroneLeaf_Networking_Configuration.md -->
## DroneLeaf — Networking Configuration (SITL)

Purpose
-------
 Summary of the networks, the IP allocation policies we follow (static vs DHCP), hostname conventions, and the minimal provisioning + tracking workflow. 

Current networks
----------------

1) Default (lab) network [Common Network]

- SSID: `Rob-Lab-C00060`
- Password: `kuri@1234!!`
- Gateway: `10.0.0.1`Inventory & hostname tracking (single source of truth)
- Netmask: `255.255.255.0`
- DHCP range: starts from `10.0.0.50` to `10.0.0.99`
- Router MAC: 'b0:19:21:27:40:d9' 

2) Deployment network [Isolated Network, usually for a single device connection]

- SSID: `DroneLeaf_Deployment`
- Password: `kuri@1234!!`
- Gateway: `192.168.144.1` (see note below)
- DHCP: `192.168.144.100`–`192.168.144.254`

High-level allocation policy (simple, consistent)
-------------------------------------------------
<!-- first for 10.0.0 network -->
- 10.0.0.x Network
    - Reserve low addresses for static assignments/reservations and infrastructure:
        - `10.0.0.1` = gateway (router)
        - `10.0.0.2`–`10.0.0.49` = static IP allocations / DHCP reservations for fixed devices (gold drones, test benches, jump boxes, servers)
        - `10.0.0.101`–`10.0.0.170` = reserved for Raspberry Pis
        - `10.0.0.171`–`10.0.0.225` = reserved for Jetson/NVIDIA devices
        - `10.0.0.226`–`10.0.0.254` = reserved for future use
    - DHCP dynamic pool: `10.0.0.50`–`10.0.0.99` (automatic assignments for new/provisioned devices)
- 192.168.144.x Network
    - Reserve low addresses for static assignments/reservations and infrastructure:
        - `192.168.144.1` = gateway (router)
        - `192.168.144.4` = reserved for px4 
        - `192.168.144.5` = reserved for companion computer
        - `192.168.144.6` = reserved for Ground Control Station
        - `192.168.144.15` = reserved for AI Computer
        - `192.168.144.25` = reserved for SiYi Camera
        - `192.168.144.100`–`192.168.144.109` = reserved for WiFi extenders
    - DHCP dynamic pool: `192.168.144.110`–`192.168.144.254` (automatic assignments for new/provisioned devices)
- Use MAC-address-based DHCP reservations in the router where possible for devices that need consistent addresses but should still boot via DHCP.

Hostname convention (organizational standard)
-------------------------------------------
Keep hostnames short, consistent and human-readable. Use this pattern:

`dl-<loc>-<role>-<nnn>`

Where:
- `dl` = DroneLeaf prefix (helps when sorting/grep)
- `<loc>` = site or lab shortcode (e.g., `rl` for Rob-Lab, `dep` for Deployment)
- `<role>` = role/type (e.g., `sitl`, `gold`, `test`, `bench`)
- `<nnn>` = three-digit numeric id, zero-padded (001, 002, ...)

Examples:
- `dl-rl-sitl-001`
- `dl-dep-gold-003`


Inventory & hostname tracking (single source of truth)
-----------------------------------------------------
- Update the network reserved devices in the following sheet: [DroneLeaf Reserved IP](https://droneleaf.sharepoint.com/:x:/s/technical/ETZjPjX8JMJOpb832DUOXkkBrR3_rG5TA8sbAhPsO_yr_Q?e=Gn4Pua) 

- Check reserved hardware from this link: [DroneLeaf Networking Inventory Sheet](https://droneleaf.sharepoint.com/:x:/s/technical/EYX_rbXUC75IigUaAJNc-XsBfMBTXl-pLNgvbqyiAvAQkw?e=fXUM90)


Contact and ownership
---------------------
For drone reserved IP, contac Abdalla Aredda (Abu Amr).