# Scope
Working with MAVLink streams from PX4 firmware and how they are feeding into/from uORB topics.

## Configure MAVLink streams

You can configure the mavlink streams from the file `src/modules/mavlink/mavlink_main.cpp` for each mode.

Under each mode you can add a message by adding a call to the `configure_stream_local` function. For example to configure the stream name `TIMESYNC` message to be sent at a 10 Hz rate you need to add `configure_stream_local("TIMESYNC", 10.0f);`



## Check MAVLink/ORB bindings and relate to ulg

### From ORB -> To MAVLink
But how to know what `configure_stream_local` stream name string should be? 

uORB topics are used in the internal PX4 communications and are captured in PX4 .ulg log files.

It is easy to trace which PX4 streamed mavlink messages correspond to which uORB messages by tracing the files in the directory `src/modules/mavlink/streams`. For example, the `SCALED_IMU3` stream has a corresponding `SCALED_IMU3.hpp` file in the `src/modules/mavlink/streams` directory. From `SCALED_IMU3` it can be seen that the message is constructed using three uORB topics: `vehicle_imu`, `vehicle_imu_status`, and `sensor_mag` enclosed by the `ORB_ID` macro. 

Use the website: https://docs.px4.io/main/en/middleware/uorb_graph.html to know who publishes and subscribes a particular ORB message.


### From MAVLink -> To ORB
To check how the MAVLink messages received by PX4 are used internally check the function `MavlinkReceiver::handle_message(mavlink_message_t *msg)` in `src/modules/mavlink/mavlink_receiver.cpp`