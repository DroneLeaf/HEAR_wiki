# Racer Project Commissioning Checklist
Version 1.0

## Form
|  |  |
| --- | --- |
| Date and time recorded | _____________________ |
| Drone ID recorded (size, IP, and other details) | _____________________ |
| Remarks and notes recorded | _____________________ |

## Early Stage Checks — Physical
### Electrical
- [x] Cables are connected
- [x] Short-circuit test passes (battery, 12V, 5V rails)
- [x] Fans are operative

### Mechanical
- [x]  No major physical damage
- [x] Backside labeled
- [x] Lock in correct position
- [x] Camera orientation is correct

## Config
### FC (Flight Controller)
- [x] MSB ports are correct (port # and baud rate)
- [x] Orientation is correct
- [x] Logs are configured-
- [x] Motor directions are correct
- [x] GPS configured and healthy (if used)

### RC
- [x] ID defined and paired
- [x] RC settings are correct (packet rate, power, Telemetry Ratio)
- [x] Model matching is ticked 

### RPI (Companion Computer)
- [x] Controller dashboard is accessible and healthy
- [x] FC and tracker versions are verified
    - leaf fc = 1.8.65
    - leaf tracker = 1.4.0
- [x] Latest DynamoDB / cloud configuration is verified (if applicable)

## Update and Maintenance Checks — Launch and connect to drone
### SSH to RPI
- [ ] Services are running
- [ ] WiFi connects
### RC Side
- [ ] RC signal forwarding works (arm check)
- [ ] RC connects (correct model selected and not in WiFi mode)

### Ground Control Side
- [x] Ground control signal is present
- [x] Telemetry data is received
- [x] Camera feed is stable and clear

### Goggles Side (if applicable)
- [x] Camera feed is stable and clear

### Stability (5 minutes)
- [x] Temperature stays below 70 °C
- [x] Tracking is active
- [x] Ground control link remains stable

## Labels
- [x] IP
- [x] Camera type
- [ ] VTX type and frequency (analog 5865hz)
- [x] ELRS number
- [x] GPS label
- [x] Software version label
- [x] Visualization is clean (unnecessary elements removed)

## Sign Off
|  |  |
| --- | --- |
| Name / Operator initials | A.H.A |
| Signature | ALMAKH |
| Date/time | 02.05.2026 |