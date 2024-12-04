# Golden Critical
GOlden prototype features plus the following:

## Triple redundant FC

- Embention
- MicroPilot

## Operation
- Payload Support (0.5kg)
- Night LEDs with front set
- Sun operation, 30°C
- Flight-time: 20 min

## Hardware: 
- Ground station
- QX1 motors and prop
- hexa
- motor signal source selection circuits.

## Avionics

- Taisync
- Optic flow
- Altimeter
- Battery SOC
- Closed RPM ESC
- triple redundant FC, Pixhawk v6x
- redundant ethernet switch.
- Gimbal with HD, SD card,
- GPS
- redundant companion computer.



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
- Gimbal trajectory
- QGC control
