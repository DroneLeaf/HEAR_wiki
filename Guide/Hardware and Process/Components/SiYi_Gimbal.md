# Setting Up SiYi Gimbal Camera with MAVLink on PX4

This guide will help you set up a SiYi gimbal camera through MAVLink with a PX4 flight controller.

## Prerequisites

- PX4 flight controller
- SiYi gimbal camera
- MAVLink communication setup
- QGroundControl or any other MAVLink-compatible ground control station

## Steps

### 1. Connect the Gimbal to the Flight Controller

1. Connect the SiYi gimbal to the PX4 flight controller using the TELEM3.
2. Ensure the gimbal is connected to the network switch properly and powered on.
3. ping 192.168.144.25 to make sure the camera is reachacble.

### 2. Configure MAVLink on PX4 for Gimbal Control

1. Open QGroundControl and connect to your PX4 flight controller (TCP: 192.168.144.5:5760).
2. Navigate to the **Vehicle Setup** page.
3. Go to the **Parameters** tab.
4. Search for `MAV_1_CONFIG` and set it to `TELEM 3` (or the appropriate port you are using).
5. Set `MAV_1_MODE` to `Gimbal` or `Normal`.
6. Set `MAV_1_FORWARD` to `ENABLED`.
7. Set `MAV_1_RATE` to `11520` (BAUD rate divided by 10).
8. Set `MAV_2_FORWARD` to `ENABLED`. (our custom instance)
9. Set `SER_TEL3_BAUD` to match the baud rate of your gimbal `115200`.
10. restart
11. Go the the parameters setup again and search for `MNT_INPUT_MODE` and set it to `MAVLink v2`.
12. D0 the same for `MNT_OUTPUT_MODE` and set it to `MAVLink v2`.
13. restart
14. In the parameters setup, you should find a tab for Component 154 (Which means gibmal is now visible to px4).
15. Inside this tab, Set `set_attr` to `1` to enable the gimbal.

### 3. Recieve Camera Feed

1. Open QGroundControl.
2. Go to the **Application Settings**.
3. Go to the **Video** tab.
4. Set the **Video Source** to `RTSP Source`.
5. Set the **RTSP URL** to `rtsp://192.168.144.25:8554/main.264`.

### 4. Control the Gimbal

1. Open QGroundControl.
2. Connect the Joystick.
3. Go to the **Vehicle Setup** page.
4. Go to the **Joystick** tab.
5. Go to the **Button Assignments** tab.
6. Assign the gimbal control to the joystick buttons (Gimbal UP, DOWN, LEFT, RIGHT, Toggle Video, Trigger Camera) and (Step Zoom in, out) to whatever you want and make sure to check `Repeat`.