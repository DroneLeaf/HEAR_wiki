# Building PX4

1. Update the file:

    Follow the instructions in [link](./modifying_mavlink_streams_and_understand_uORB_bindings.md#From-ORb-->-Ulog) for details on adding new topics.

    ```
    src/modules/logger/logged_topics.cpp
    ```
2. Run the dependency script:
    ```
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
    ```

3. Build the firmware:
    ```
    make px4_fmu-v6x_default
    ```