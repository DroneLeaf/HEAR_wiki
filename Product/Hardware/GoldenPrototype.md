# Golden Prototype

## Aim
- To have a good flying multirotor that we can always show to clients and complete certain standard client requirements with.

## Operation
- Payload Support (1kg)
- Night LEDs with front set
- Sun operation, 30°C
- Flight-time: 20 min

## Hardware: 
- Ground station
- QX1 motors and prop
- quad

## Avionics

- Taisync
- Optic flow
- Altimeter
- Battery SOC
- Closed RPM ESC (Needs review)
- Pixhawk v6x
- Eth Switch
- Gimbal with HD, SD card,
- GPS
- Jetson Orin



## Software Safety

- Angle limit at take-off/land
- Ground detection
- Dynamic stop trajectory
- Smooth bias elimination
- Persistent memory for biases
- Pre-flight checks: 
  - GPS
  - PX4
  - Motor RPM + current
  - Sensors valid
  - RC kill switch
  - Ground station
  - Heading pre-known
  
- Emergency landing: 
  - GPS
  - Battery SOC

## Mission

- Yaw trajectory
- Home position
- GPS waypoint support
- Gimbal control
- QGC control
