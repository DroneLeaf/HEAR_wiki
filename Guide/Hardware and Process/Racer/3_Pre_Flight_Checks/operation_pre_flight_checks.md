# Racer Project Operation Pre-flight Checks
Version 1.0

## Labels
- [ ] IP
- [ ] Camera type
- [ ] VTX type and frequency
- [ ] ELRS number
- [ ] GPS label
- [ ] Software version label
- [ ] Visualization is clean (unnecessary elements removed)

## Operation Pre-flight Checks
- [ ] No physical damage
- [ ] ELRS LED is solid red
- [ ] Battery full
- [ ] Props on, correct direction, and secure (quick arm check)
- [ ] RPI is pingable
- [ ] Do calibration (Gyro)
- [ ] Check Video feed (Goggles)
- [ ] Tracking feed is stable
- [ ] GPS signal is received (for GPS logging)
- [ ] Check that the selected profile uuid in `config-robot_instance_profile_assignment` matches `Indoor-Testing` profile in `config-profile`
- [ ] Controller dashboard shows no missing database fields in `http://Drone IP Address/home/leaf-racer-configuration`
  - [ ] Click "Get Configuration" and check for any missing fields. If there are missing fields, update the configuration files in `config-leaf_tracker` and `config-guidance_by_vision` according to the reference values in [config-guidance_by_vision](config-guidance_by_vision) and [config-leaf_tracker](config-leaf_tracker), then click "Upload Configuration" to update the drone's configuration.
  - [ ] Check "Hardware Setup"
    - Racer System Status Publish Rate: **5 Hz**
    - Betaflight Status Publish Rate 1 - 2 Hz
    - Betaflight MSP Port is set to **/dev/ttyAMA0** at baud rate **230400**
    - Betaflight ELRS Port is set to **/dev/ttyAMA1** at baud rate **420000**
    - Betaflight ELRS Secondary TX Port is **empty**
    - Betaflight ELRS-to-Ground Port is set to **/dev/ttyAMA2** at baud rate **420000**
  - [ ] Check "Camera Mount Configuration"

    | Camera     | Twist | Video Source Type | Video Source Index / Link                 | Blocked video sources        |
    |------------|-------|-------------------|-------------------------------------------|------------------------------|
    | Foxeer 90  | 0     | Analog            | 0                                         | 10,11,12,13,14,15,16         |
    | Foxeer 120 | 0     | Analog            | 0                                         | 10,11,12,13,14,15,16         |
    | Runcam     | -90   | Digital           | `rtsp://root:12345@192.168.1.10/stream=1` | N/A                          |

  - [ ] Check "Tracker Configuration"
    - Clickbox Size (pixels): 25 - 25 pixels (for all cameras)

    | Camera     | HFOV | VFOV |
    |------------|------|------|
    | Foxeer 90  | 90   | 70   |
    | Foxeer 120 | 120  | 95   |
    | Runcam     | 135  | 83   |

    - "Tracker algorithm" is set to **"csrt_fast"**
    - "Use contour detection" is **enabled** ✅
    - "Disable robustness measures" is **disabled** ❌
    - Scaling Recheck Interval is set to **0** (disabled) ⚠️
    - "Auto recover" is **enabled** ✅
    - "Recovery History" is set to **10 frames**
    - "Recovery search scale" is set to **0.3** (30 % of the frame)
    - "Recover Full Frame" is disabled ❌
    - "Minimum box size" is set to **2 pixels**
    - "Keep seeded on loss" is **enabled** ✅
    - "Edge margin" is set to **0** ⚠️
    - "Max Frame Fill" is set to **1.0** (100 % of the frame)
    - "Max Jump" is set to **0.5** (50 % of the frame)
    - "Init Padding" is set to **0**
    - Under "CSRT Advanced Settings":
      - "PSR Threshold" is set to **0.035**
      - "Use Color Names" is **enabled** ✅
      - "Use RGB" is **enabled** ✅
      - "User Segmentation" is **disabled** ❌
      - "Padding" is set to **2**
      - "Template Size" is set to **100 pixels**
      - "HOG Channels Used" is set to **9**
      - "Number of Scales" is set to **7**
      - "ADMM Iterations" is set to **1**
      - "Use HOG" is **enabled** ✅
      - "HOG Iterations" is set to **6**
    - Under "Advanced Tracker Settings":
      - "Status Publish Interval" is set to **0.1 seconds** (10 Hz)
      - "State Publish Interval" is set to **0.1 seconds** (10 Hz)
      - "Redis DB" is set to **0**
      - "Redis Unix Socket Path" is set to `/var/run/redis/redis-server.sock`
      - **Disable** "Dark-Frame Check" ❌
  - [ ] Check "Contour Detection" settings

    | Camera     | Contour Profile                  |
    |------------|----------------------------------|
    | Foxeer 90  | Dark Sky / Light Object          |
    | Foxeer 120 | Dark Sky / Light Object          |
    | Runcam     | Light Sky / Dark Object (Digital)|

  - [ ] Check "Interceptor Settings"
      - "Use Gravity Compensation Guidance" is **selected** ✅
      - "Manual - Ramp to Abs. Target" is **selected** ✅
      - "Automatic Takeoff" is **selected** ✅
      - "Activate Gravity Compensation" is **enabled** ✅

      | Parameter                                | 5" Drone | 7" Drone |
      |------------------------------------------|----------|----------|
      | Activate Gravity Compensation            | ✅       | ✅       |
      | Minimum Hover Throttle for Gravity Comp. | 0.15     | 0.1      |
      | Direct Angle Tracking                    | ❌       | ❌       |
      | Angle Error Correction Kp                | 1        | 1        |
      | Max Angle Increment Roll (rad)           | 3        | 3        |
      | Max Angle Increment Pitch (rad)          | 3        | 3        |
      | Max Accumulated Angle Error Roll (rad)   | 0.262    | 0.262    |
      | Max Accumulated Angle Error Pitch (rad)  | 0.262    | 0.262    |
      | Minimum Target Interception Throttle     | 0.05     | 0.05     |
      | Takeoff Ramp Time (s)                    | 0.2      | 0.2      |
      | Takeoff Wait Time (s)                    | 0.4      | 0.5      |
      | Hover Throttle for Takeoff Logic         | 0.3      | 0.45     |

  - [ ] Check "RC Calibration Settings"
    - Attitude Axis Mapping (RC → Angle)

      | Axis  | RC Min | RC Max | Out Min | Out Max |
      |-------|--------|--------|---------|---------|
      | Roll  | 1000   | 2000   | -70°    | 70°     |
      | Pitch | 1000   | 2000   | -70°    | 70°     |
      | Yaw   | 1000   | 2000   | -1      | 1       |

      - ⚠️ **IMPORTANT:** Roll and Pitch mapping Out Min/Max degrees **must match** the `angle_limit` setting in Betaflight Configurator (typically **70°** for 5" drones and **80°** for 7" drones).

    - Normalized Command Mapping

      | Axis     | RC Min | RC Max | Out Min | Out Max |
      |----------|--------|--------|---------|---------|
      | Aim X    | 988    | 2012   | 0       | 1       |
      | Aim Y    | 988    | 2012   | 0       | 1       |
      | Box Size | 988    | 2012   | 0       | 1       |
      | Thrust   | 1000   | 2000   | 0       | 1       |


## Launch and connect to drone
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