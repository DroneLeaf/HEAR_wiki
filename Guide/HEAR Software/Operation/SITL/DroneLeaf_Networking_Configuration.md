## DroneLeaf — Networking Configuration (SITL)

Purpose
-------
This document provides a concise, organizational-standard summary of the networks, the IP allocation policies we follow (static vs DHCP), hostname conventions, and the minimal provisioning + tracking workflow. 

Use the inventory sheet (oneDrive.com) as the single source of truth for all device assignments.

Current networks
----------------

1) Default (lab) network [Common Network]

- SSID: `Rob-Lab-C00060`
- Password: `kuri@1234!!`
- Gateway: `10.0.0.1`
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

dl-<loc>-<role>-<nnn>

Where:
- `dl` = DroneLeaf prefix (helps when sorting/grep)
- `<loc>` = site or lab shortcode (e.g., `rl` for Rob-Lab, `dep` for Deployment)
- `<role>` = role/type (e.g., `sitl`, `gold`, `test`, `bench`)
- `<nnn>` = three-digit numeric id, zero-padded (001, 002, ...)

Examples:
- `dl-rl-sitl-001`
- `dl-dep-gold-003`

Machine-readable regex (for validation):
```
^dl-[a-z0-9]{1,8}-(sitl|gold|test|bench)-[0-9]{3}$
```

Inventory & hostname tracking (single source of truth)
-----------------------------------------------------IP
- Update the network reserved devices in the following sheet: [DroneLeaf Reserved IP] (https://droneleaf.sharepoint.com/:x:/s/technical/ETZjPjX8JMJOpb832DUOXkkBrR3_rG5TA8sbAhPsO_yr_Q?e=Gn4Pua) 

- Check reserved hardware from this link: [DroneLeaf Networking Inventory Sheet](https://droneleaf.sharepoint.com/:x:/s/technical/EYX_rbXUC75IigUaAJNc-XsBfMBTXl-pLNgvbqyiAvAQkw?e=fXUM90)


Maintain a simple CSV as the authoritative inventory. Prefer a shared Google Sheet if multiple people will edit concurrently. The CSV headers we recommend:

hostname,mac,ip,gateway,netmask,ssid,assigned_by,date,location,notes

Example row:

dl-rl-sitl-012,AA:BB:CC:11:22:33,10.0.0.12,10.0.0.1,255.255.255.0,Rob-Lab-C00060,mike,2025-11-09,Rob-Lab,Initial provisioning

Practical rules for the inventory
- Use MAC as the immutable identifier for a device.
- Always update the inventory immediately after assigning a new hostname/IP.
- If you create a DHCP reservation, note reservation details and the router used.

Collision avoidance
-------------------
- Before assigning a static IP, check the DHCP leases on the router and the inventory CSV.
- Where possible, prefer adding a DHCP reservation (router) rather than configuring static IPs on devices — this centralizes control and reduces mistakes.

Security and credentials
------------------------
- The default Wi‑Fi password listed here is operational; rotate it as part of your security hygiene and keep credentials in a secure vault (1Password/Bitwarden or company secrets manager) rather than in plain docs when possible.

Simple checklist for provisioning a new drone
-------------------------------------------
1. Connect to correct SSID.
2. Note MAC and DHCP-assigned IP.
3. Pick or confirm hostname (use pattern above).
4. Set hostname on device.
5. Decide: DHCP reservation (preferred) or manual static.
6. Update inventory CSV/Sheet immediately.
7. Verify reachability (ping/ssh) and DNS/hosts if used.

Next steps / recommendations
----------------------------
- Keep this doc short and link to the inventory CSV/Sheet from here.
- Add a small script to validate inventory vs router leases (optional). A simple script can parse the router's DHCP lease file and the CSV to flag mismatches.
- Consider a small Ansible playbook or provisioning script to set hostname and register the device automatically into the inventory for larger scale.

Small example CSV header (copyable)
```
hostname,mac,ip,gateway,netmask,ssid,assigned_by,date,location,notes
```

Contact and ownership
---------------------
Maintain an owner for the inventory and networking setup in the team (name/email). This owner approves IP changes and performs periodic audits (quarterly).

Change log
----------
- 2025-11-09 — Created initial organizational-standard summary and inventory schema. Assumed `192.168.144.1` gateway for `DroneLeaf_Deployment` due to DHCP range; please confirm if the `192.164` address was intentional.

