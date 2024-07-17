# Stream rates settings are overriden by QGroundControl at startup, see:

https://discuss.ardupilot.org/t/requesting-stream-rates-using-mavros-qgc-srx-params-etc/86008/5
https://github.com/mavlink/qgroundcontrol/blob/master/src/FirmwarePlugin/APM/ArduSubFirmwarePlugin.cc#L153


# Setting MAVLink router to allow multiple programs to listen at the same UDP port (e.g. QGC + HEAR_FC)

mavlink router setup guide: https://github.com/mavlink-router/mavlink-router

sudo mavlink-routerd -e 127.0.0.1:14551 -e 192.168.2.1:14552  0.0.0.0:14550

The 14551 port is for QGC, a connection needs to be added with these settings in QGC App settings.
The 14552 port is for HEAR_FC, HEAR_Configurations needs to be setup accordingly.


# Add MAVLink support in wireshark
Copy the generated WLua file to the wireshark plugins folder:

sudo cp WLua.lua /usr/lib/x86_64-linux-gnu/wireshark/plugins/3.2/epan/WLua.lua