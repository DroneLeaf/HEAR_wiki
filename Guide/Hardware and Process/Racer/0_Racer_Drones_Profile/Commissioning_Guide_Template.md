# Racer Project Commissioning Checklist
Version 1.1

## Form
|  |  |
| --- | --- |
| Date and time recorded | _____________________ |
| Drone ID recorded (size, IP, and other details) | _____________________ |
| Remarks and notes recorded | _____________________ |

## Early Stage Checks — Physical
### Electrical
- [ ] Cables are connected
- [ ] Short-circuit test passes (battery, 12V, 5V rails)
- [ ] Fans are operative

### Mechanical
- [ ] No major physical damage
- [ ] Backside labeled
- [ ] Lock in correct position
- [ ] Camera orientation is correct

## Config
### FC (Flight Controller)
- [ ] MSB ports are correct (port # and baud rate)
- [ ] Orientation is correct
- [ ] Logs are cleared & SD is empty
- [ ] Motor directions are correct
- [ ] GPS configured and healthy (if used)

### RC
- [ ] ID defined and paired
- [ ] RC settings are correct (packet rate, power, Telemetry Ratio)
- [ ] Model matching is ticked 

### RPI (Companion Computer)
- [ ] Controller dashboard is accessible and healthy
- [ ] FC and tracker versions are verified
- [ ] Latest DynamoDB / cloud configuration is verified (if applicable)

## Update and Maintenance Checks — Launch and connect to drone
### SSH to RPI
- [ ] Services are running
- [ ] WiFi connects
### RC Side
- [ ] RC signal forwarding works (arm check)
- [ ] RC connects (correct model selected and not in WiFi mode)

### Ground Control Side
- [ ] Ground control signal is present
- [ ] Telemetry data is received
- [ ] Camera feed is stable and clear

### Goggles Side
- [ ] Camera feed is stable and clear

### Stability (5 minutes)
- [ ] Temperature stays below 70 °C
- [ ] Tracking is active
- [ ] Ground control link remains stable

## Labels
- [ ] IP
- [ ] Camera type
- [ ] VTX type and frequency
- [ ] ELRS number
- [ ] GPS label
- [ ] Software version label
- [ ] Visualization is clean (unnecessary elements removed)

## Sign Off
|  |  |
| --- | --- |
| Name / Operator initials | _____________________ |
| Signature | _____________________ |
| Date/time | _____________________ |