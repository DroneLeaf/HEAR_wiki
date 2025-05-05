## System-wide replay feature in PX4

First, choose the file to replay and build the target (from within the `PX4-Autopilot` directory):

```bash
export replay=<absolute_path_to_log_file.ulg>
make px4_sitl_default
```

This will create the `build/make` output in a separate build directory `build/px4_sitl_default_replay` (so that the parameters don't interfere with normal builds). It's possible to choose any posix SITL build target for replay, since the build system knows through the replay environment variable that it's in replay mode.

Add ORB publisher rules in the file b`uild/px4_sitl_default_replay/rootfs/orb_publisher.rules`. This file defines the modules that are allowed to publish particular messages. It has the following format:

```yaml
restrict_topics: <topic1>, <topic2>, ..., <topicN>
module: <module>
ignore_others: <true/false>
```

This means that the given list of topics should only be published by `<module>` (which is the command name). Publications to any of these topics from another module are silently ignored. If ignore_others is true, publications to other topics from `<module> `are ignored.

For replay, we only want the replay module to be able to publish the previously identified list of topics. So, for replaying `ekf2`, the rules file should look like this:

```yaml
restrict_topics: sensor_combined, vehicle_gps_position, vehicle_land_detected
module: replay
ignore_others: true
```

With this, the modules that usually publish these topics don't need to be disabled for the replay.

*(Optional)* Copy a dataman mission file from the SD card to the build directory. This is only necessary if a mission should be replayed.

Start the replay:

```bash
make px4_sitl_default jmavsim
```

This will automatically open the log file, apply the parameters and start the replay. Once done, it will report the outcome and exit. The newly generated log file can then be analyzed. It can be found in `rootfs/fs/microsd/log`, in subdirectories organised by date. Replayed log file names will have the `_replayed` suffix.

Note that the above command will show the simulator as well, but - depending on what is being replayed - it will not show what's actually going on. It is still possible to connect via QGC and, for example, view the changing attitude during replay.

## Setup parameter overrides

By default, all parameters from the original log file are applied during a replay. If a parameter changes during recording, it will be changed at the right time during the replay.

Parameters can be overridden during a replay in two ways: fixed and dynamic. When parameters are overridden, corresponding parameter changes in the log are not applied during replay.

Fixed parameter overrides will override parameters from the start of the replay. They are defined in the file `build/px4_sitl_default_replay/rootfs/replay_params.txt`, where each line should have the format `<param_name> <value>`. For example:

```c
EKF2_RNG_NOISE 0.1
```

Dynamic parameter overrides will update parameter values at specified times. These parameters will still be initialised to the values in the log or in the fixed overrides. Parameter update events should be defined in b`uild/px4_sitl_default_replay/rootfs/replay_params_dynamic.txt`, where each line has the format `<param_name> <value> <timestamp>`. The timestamp is the time in seconds from the start of the log. For example:

```c
EKF2_RNG_NOISE 0.15 23.4
EKF2_RNG_NOISE 0.05 56.7
EKF2_RNG_DELAY 4.5 30.0
```

> [!TIP]
> You may export the list of existing parameters using `pyulog` and then modifiying them in place as explained in [this section](#adjusting-ekf2-specific-parameters-for-the-replay)

Once finished, unset `replay`.

```bash
unset replay
```

## Adjusting EKF2-specific Parameters for the Replay

First install `pyulog`:

```bash
pip install --user pyulog
```

Extract the original log's parameters to replay_params.txt:

```bash
ulog_params -i "$replay" -l ' ' | grep -e '^EKF2' > build/px4_sitl_default_replay/rootfs/replay_params.txt
```

Adjust these as desired, and add dynamic parameter overrides in `replay_params_dynamic.txt` if necessary.

## EKF2 single instance replay 

This is a specialization of the system-wide replay for fast EKF2 replay.

> [!NOTE]  
> The recording and replay of flight logs with multiple EKF2 instances is not supported. To enable recording for EKF replay you must set the parameters to enable a single EKF2 instance as explained in [EKF2_replay_logging_setup](../External%20Software/PX4/EKF2_replay_logging_setup.md)

In EKF2 mode, the replay will automatically create the ORB publisher rules described above.

To perform an EKF2 replay:

Record the original log. Optionally set `SDLOG_MODE` to `1` to log from boot.

In addition to the replay environment variable, set `replay_mode` to ekf2:

```bash
export replay_mode=ekf2
export replay=<absolute_path_to_log.ulg>
```

Run the replay with the none target:

```
make px4_sitl none
```

Once finished, unset both `replay` and `replay_mode`.

```bash
unset replay; unset replay_mode
```

## Important notes

> [!WARNING]  
> During replay, all dropouts in the log file are reported. These have a negative effect on the replay, so care should be taken to avoid dropouts during recording.

> [!NOTE]  
> It is currently only possible to replay in 'real-time': as fast as the recording was done. This is planned to be extended in the future.

> [!NOTE]  
> A message that has a timestamp of 0 will be considered invalid and not be replayed.

> [!Caution]  
> Currently the EKF2 replay feature shows discrepancies against the original log file. The core team is still investigating.