# Gimbal and Camera Control Specifications

## Gimbal features:

- Goto a specific attitude
- Manual control of gimbal attitude
- Yaw-lock mode / Yaw-follow mode. (See https://ardupilot.org/dev/docs/mavlink-gimbal-mount.html#mav-cmd-do-gimbal-manager-pitchyaw-to-move-to-a-desired-angle-or-at-a-desired-rate)
- Look to a specific GPS point (https://ardupilot.org/dev/docs/mavlink-gimbal-mount.html#mav-cmd-do-set-roi-location-to-point-at-a-location)
- Go-to-home position


## Camera features
- Zoom in/out by manual control
- Take a photo
- Video start/stop
- Real-time streaming through RTSP or other integrable protocols

## Integration
- MAVLink gimbal+camera control protocol
- Support for LeafFC pipelines and QGC operation