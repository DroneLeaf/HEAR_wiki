# Overview


## PX4 Connectivity
Check leafFC MonitorPX4Status_Block implementation

## QGC Connectivity
Dump QGC heartbeat (enabled by default from application settings):
```
MAVLink Protocol (21)
    Header
        Magic value / version: MAVLink 2.0 (0xfd)
        Payload length: 9
        Incompatibility flag: 0x00 (0)
        Compatibility flag: 0x00 (0)
        Packet sequence: 45
        System id: 255
        Component id: MAV_COMP_ID_MISSIONPLANNER (190)
        Message id: HEARTBEAT (0)
    Payload: HEARTBEAT (0)
        type (MAV_TYPE): MAV_TYPE_GCS (6)
        autopilot (MAV_AUTOPILOT): MAV_AUTOPILOT_INVALID (8)
        base_mode (MAV_MODE_FLAG): 0xc0 (192)
            .... ...0 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED: False
            .... ..0. = MAV_MODE_FLAG_TEST_ENABLED: False
            .... .0.. = MAV_MODE_FLAG_AUTO_ENABLED: False
            .... 0... = MAV_MODE_FLAG_GUIDED_ENABLED: False
            ...0 .... = MAV_MODE_FLAG_STABILIZE_ENABLED: False
            ..0. .... = MAV_MODE_FLAG_HIL_ENABLED: False
            .1.. .... = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED: True
            1... .... = MAV_MODE_FLAG_SAFETY_ARMED: True
        custom_mode (uint32_t): 0
        system_status (MAV_STATE): MAV_STATE_ACTIVE (4)
        mavlink_version (uint8_t): 3
    Message CRC: 0xce53

```

## RC Connectivity
Check: https://mavlink.io/en/messages/common.html#RC_CHANNELS_RAW

field `rssi` value not `UINT8_MAX`

## Mag connectivity
Investigate the use of:
https://mavlink.io/en/messages/common.html#HIGHRES_IMU_UPDATED_FLAGS

