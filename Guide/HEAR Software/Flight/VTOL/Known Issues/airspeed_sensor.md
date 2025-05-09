# Integration of UAVCAN airspeed sensor

## Symptoms and Root cause analysis
See attached `communication_with_cuav.pdf` 

## Workaround
add `airspeed_selector start` to the startup script. Nothing for consideration from the operator side.

Link: (KU Snono Startup script)[https://github.com/DroneLeaf/PX4-Autopilot/blob/leaf-main/ROMFS/px4fmu_common/init.d/airframes/4999_snono_vtol]

## QGC side fix (unsuccessful):
Edited in `src/AutoPilotPlugins/PX4/SensorsComponent.cc` the line `if (_vehicle->fixedWing() || _vehicle->vtol() || _vehicle->airship()) {` to be `if (_vehicle->fixedWing() || _vehicle->vtol() || _vehicle->airship() || _vehicle->multiRotor()) {`

Still not working. Seems to be an issue with PX4 1.14.3, see:

https://github.com/mavlink/qgroundcontrol/issues/11882#issue-2527014585

**Confirmed**: Upgrading to 1.15.4 solved the issue and SYS_HAS_NUM_ASPD parameter appears in QGC

## Calibration workaround
**Option 1 (Recommended)**: To do manual calibration by modifying `SENS_DPRES_OFF` for offset cancellation followed by `ASPD_SCALE_1` modification to map the IAS to CAS.

**Option 2**: To do manual calibration and apply them post-flight at the log analysis stage.