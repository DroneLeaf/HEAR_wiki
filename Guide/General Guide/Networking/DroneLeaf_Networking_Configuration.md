<!-- file name: DroneLeaf_Networking_Configuration.md -->
## DroneLeaf — Networking Configuration (SITL)

### Scope
- Networks, IP pools, hostnames, tracking owners.

### Networks
| Network | Use | SSID | Passphrase | Gateway | Netmask | DHCP Pool |
| --- | --- | --- | --- | --- | --- | --- |
| Rob-Lab* 10.0.0.x | shared lab | `Rob-Lab-C00060` | `kuri@1234!!` | `10.0.0.1` | `255.255.255.0` | `10.0.0.50–99` |
| DroneLeaf_Deployment 192.168.144.x | isolated field | `DroneLeaf_Deployment` | `kuri@1234!!` | `192.168.144.1` | `255.255.255.0` | `192.168.144.110–254` |

### Allocation Bands
**10.0.0.x (Lab)**
| Range | Use |
| --- | --- |
| `1` | Gateway — MAC `b0:19:21:27:40:d9` |
| `2–49` | Static / infra |
| `50–99` | DHCP pool |
| `101–170` | Raspberry Pi |
| `171–225` | Jetson / NVIDIA |
| `226–254` | Spare |

**192.168.144.x (Deployment)**
| Range | Use |
| --- | --- |
| `1` | Gateway |
| `4` | PX4 |
| `5` | Companion computer |
| `6` | Ground Control Station |
| `15` | AI computer |
| `25` | SiYi camera |
| `100–109` | Wi-Fi extenders |
| `110–254` | DHCP pool |

- Favor MAC-based DHCP reservations for persistent nodes.

### Hostnames
- Pattern `dl-<loc>-<role>-<nnn>`
- Loc: `rl` (Rob-Lab), `dep` (Deployment) …
- Role: `sitl`, `gold`, `test`, `bench` …
- `nnn`: zero-padded device id (001+)

### Tracking & Ownership
- Update [DroneLeaf Reserved IP](https://droneleaf.sharepoint.com/:x:/s/technical/ETZjPjX8JMJOpb832DUOXkkBrR3_rG5TA8sbAhPsO_yr_Q?e=Gn4Pua).
- Audit hardware via [Networking Inventory](https://droneleaf.sharepoint.com/:x:/s/technical/EYX_rbXUC75IigUaAJNc-XsBfMBTXl-pLNgvbqyiAvAQkw?e=fXUM90).
- Owner: Abdalla Aredda (Abu Amr).
