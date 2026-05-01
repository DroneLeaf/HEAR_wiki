# Racer Project Pre-Flight Checklist

## Form
- [ ] Date and time recorded
- [ ] Drone ID recorded (size, IP, and other details)
- [ ] Remarks and notes recorded

## Physical
- [ ] Cables are connected
- [ ] Short-circuit test passes (battery, 12V, 5V rails)
- [ ] No major physical damage
- [ ] Backside labeled
- [ ] Lock in correct position
- [ ] Camera orientation is correct
- [ ] Fans are operative

## Config
### FC
- [ ] MSB ports are correct (port # and baud rate)
- [ ] Orientation is correct
- [ ] Logs are configured/enabled
- [ ] Motor directions are correct
- [ ] GPS is configured (if used)

### RC
- [ ] ID defined and paired
- [ ] RC settings are correct
- [ ] Model matching is ticked

### RPI
- [ ] Controller dashboard is accessible and healthy
- [ ] FC and tracker versions are verified
- [ ] Latest DynamoDB configuration is verified

## Running
### Launch
- [ ] Services are running
- [ ] WiFi connects
- [ ] Video feed check passes
- [ ] RC signal forwarding works (arm check)
- [ ] Ground control signal is present
- [ ] RC connects (correct model selected and not in WiFi mode)
- [ ] Video feed is stable and clear [OSD is correct, if used]

### Stability (5 minutes)
- [ ] Temperature stays below 70 C
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

## Pre-flight
- [ ] ELRS LED is solid red
- [ ] Battery full
- [ ] Props on, correct direction, and secure (quick arm check)
- [ ] RPI is pingable, services are running
- [ ] Do calibration (Gyro)
- [ ] Check Video feed (Goggles)
- [ ] Tracking feed is stable
- [ ] GPS signal is received (for GPS logging)

## Sign Off
- [ ] Name
- [ ] Signature