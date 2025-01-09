
## External mag connectivity check
Check SCALED_IMU3 https://mavlink.io/en/messages/common.html#SCALED_IMU3

xmag ymag and zmag not zeros

Added these lines to PX4 firmware (mavlink_main.cpp):

```
		// Additional for sensors validity check
		configure_stream_local("SCALED_IMU",  2.0f);
		configure_stream_local("SCALED_IMU2", 2.0f);
		configure_stream_local("SCALED_IMU3", 2.0f);
```

## ARKFlow connectivity checks
Check this mavlink message: https://mavlink.io/en/messages/common.html#OPTICAL_FLOW

Check if the field ground_distance value is positive

