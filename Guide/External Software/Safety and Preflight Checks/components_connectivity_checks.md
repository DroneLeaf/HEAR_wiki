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
        Incompatibility flag: 0
        Compatibility flag: 0
        Packet sequence: 79
        System id: 0xff
        Component id: 0xbe
        Message id: HEARTBEAT (0)
    Payload: HEARTBEAT (0)
    Message CRC: 0x0761
```

## RC Connectivity
Check: https://mavlink.io/en/messages/common.html#RC_CHANNELS_RAW

field `rssi` value not `UINT8_MAX`

## Mag connectivity
Investigate the use of:
https://mavlink.io/en/messages/common.html#HIGHRES_IMU_UPDATED_FLAGS

