# Mavlink Messages Send/Receive
The goal of this guide is to show you how to send and receive MAVLink messages using the generated MAVLink definitions in python to make the testing and debugging process easier for core developers.

## 1. Generate the MAVLink Definitions for Python
Navigate to the mavlink repo root folder in your repo (LeafMC, LeafFC) and run the following command to generate the MAVLink definitions for python:
```bash
python3 -m mavgenerate
```
- It will open a gui, select the all.xml dialict from the message_definitions/folder and select the output folder to be the pymavlink in the mavlink folder.
- Choose the language to be Python and protocol version 2.0 then click Generate.
- A pymavlink.py file will be generated in the mavlink folder.
- move this file to the $HOME directory and rename it to MAV.py
    ```bash
    mv -f pymavlink.py $HOME/MAV.py
    ```
- Use the bellow testMav.py script as an example to use the messeges in your python script.
    ```python
    #!/bin/python3

    import sys, time
    # Import mavutil
    import MAV
    from pymavlink import mavutil
    # print(mavutil.__file__)

    # Create the connection
    master = mavutil.mavlink_connection('tcp:0.0.0.0:5760', source_system=0)

    # Choose a mode
    mode = MAV.LEAF_MODE_LEARNING_FULL
    status = MAV.LEAF_STATUS_READY_TO_FLY

    # Request all parameters
    master.mav.param_request_list_send(
        master.target_system, master.target_component
    )

    client_msg = MAV.MAVLink_leaf_client_tagname_message(b"ENEC")
    master.mav.send(client_msg)
    while True:
        time.sleep(0.1)
        mode_msg = MAV.MAVLink_leaf_mode_message(mode)
        status_msg = MAV.MAVLink_leaf_status_message(status)
        master.mav.send(mode_msg)
        master.mav.send(status_msg)
        
        try:
            message = master.recv_match(type=MAV.MAVLINK_MSG_ID_LEAF_SET_MODE,  blocking=False)
            if message is not None:
                print(message)
        except Exception as error:
            print(error)
    ```
Run this script with 
```bash
python3 -m mavgenerate
```

and mavlink-routerd:
```
mavlink-routerd -e 127.0.0.1:14650  0.0.0.0:14550 # PX4 SITL example
```

 and then it should populate mavlink messages on the network so your FC or MC can capture them and you will be able to test and debug the behavior of your code.

 See the guide on Wireshark