# Features of Launcher V2

- Supports launching of 5-inch drones with pan/tilt mechanism
- Supports launching of 7-inch drones with pan/tilt mechanism
- Order of system rotation is pan then tilt.
- Operations in temperatures of up to 50 degrees
- Operations in sandy environments
- Operations under direct sunlight
- Dock with doors
- IP rating of 54
- Attachable to a pick-up vehicle

## Scope of contractor
- Design assistance for the electromechanical aspects.
- Procurement, fabrication, assembly and design verification.

## Out of Scope of contractor
- Software aspects except what is required for functional verification.

## Design specifications for launcher V2

### Motion
- Pan range is preferably unlimited. Next best is 360 degrees. Minimum acceptable is 180 degrees.
- Tilt range is 90 degrees to 65 degrees. This corresponds to a drone tilt of 45 degrees from zenith when tilt is 90 degrees.
- Method to determine pan/tilt angles and limit the motion. e.g. absolute encoders provide feedback about both pan and tilt. They must be also used to limit motion.
- Suggested to use BLDC actuation similar to the ones used in robots and robodogs. This would allow tracking of moving targets.
- The drone is covered by dock doors. Transition from closed to open position must be fastest possible, and not more than 5 seconds.

### Mechanical and Thermal
- Majorly aluminum. Use extruded aluminum. Make as light as possible. Preferably possible to carry by a single person, worst case by two person.
- Dimensions must not exceed 60Wx60Lx80H cm. Open for revision for small expansions in dimension.
- Color: Desert Sand
- The drone needs to be cooled down to below 25 deg C when the dock is closed. The drone must have a mechanism to keep the drone cooled down after the dock doors open for up to 3 minutes.

### Electrical
- The launcher is battery powered. The battery must support 8h including 4hr of active cooling.
- The launcher is chargeable from 220v power source.
- Good to have: hot-swap support for the battery.

### Ejection
#### Mechanical
- The drone slides through the pipe rails. 7-inch has dual rail, 5-inch has single rail.

#### Electrical
- The drone is tethered to the launcher with power cable and signal cable (UART or ethernet, tbd).
- The tether disconnects effortlessly when the drone launches.
- The tether can be connected again to re-establish communication and power.
- The power charges the battery to its max voltage and supplies drone electronics to avoid draining drone battery. Nominal current draw is 3A, voltage must be adjustable from 25V to 60V. Tip: Battery balance charge cable could be used.

### Communication
- Ethernet reel or fiber-optic reel for IP connection with a length of at least 50m. Easy to deploy and retract. The cable links the launcher to the operator station. Review the possibility of using PoE.
- Alternative wireless communication to the operator station must be employed. 

### User Interface
- The dock must show voltage display, on the device or some sort of low-voltage warning.
- The dock must employ emergency shut-down switch.
- The dock must have clear indicator when in active operation to warn personnel from being at the vicinity.
- The dock must report over the communication medium to the operator station: battery info, gimbal pose, and full status.
- The dock must accept over the communication medium from the operator station: gimbal control commands, door open/close commands and other major commands.
- The contractor advises on additional possible ideas that improves user experience.

## Some recommended components
### Actuators for Pan/Tilt
https://ar.aliexpress.com/item/1005012000854335.html?spm=a2g0o.detail.pcDetailTopMoreOtherSeller.2.27d3iu9giu9gDE&gps-id=pcDetailTopMoreOtherSeller&scm=1007.40050.354490.0&scm_id=1007.40050.354490.0&scm-url=1007.40050.354490.0&pvid=4847a9d4-b4bf-4880-80df-58a712f23f6e&_t=gps-id%3ApcDetailTopMoreOtherSeller%2Cscm-url%3A1007.40050.354490.0%2Cpvid%3A4847a9d4-b4bf-4880-80df-58a712f23f6e%2Ctpp_buckets%3A668%232846%238108%231977&pdp_ext_f=%7B%22order%22%3A%22-1%22%2C%22spu_best_type%22%3A%22price%22%2C%22eval%22%3A%221%22%2C%22sceneId%22%3A%2230050%22%2C%22fromPage%22%3A%22recommend%22%7D&pdp_npi=6%40dis%21AED%21941.61%21856.87%21%21%211699.35%211546.41%21%400b1bf20817777376672671902ebfb1%2112000057252322566%21rec%21AE%21%21ABX%211%210%21n_tag%3A-29910%3Bd%3A3e5c86c5%3Bm03_new_user%3A-29895&utparam-url=scene%3ApcDetailTopMoreOtherSeller%7Cquery_from%3A%7Cx_object_id%3A1005012000854335%7C_p_origin_prod%3A#nav-specification 

### Communication
- Taisync DUO for wireless ethernet communication