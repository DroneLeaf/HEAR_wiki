# The Estimator replay (EKF2) feature in PX4

We can use the EKF2 replay feature to diagnose sensor loss issues. This logs all sensor outputs at the full rate allowing us to replay the estimator output offline post-flight. For details please consult the PX4 documentation [here](https://docs.px4.io/main/en/debug/system_wide_replay.html#ekf2-replay).

To succesfully capture a log file suitable for EKF2 replay we need to do the following:

* Need to log at full rate using EKF2_replay profile by setting `SDLOG_PROFILE` to `1` for Estimator replay (EKF2).
* Have to turn off multi-EKF off. See [here](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html#running-a-single-ekf-instance).

    The parameter settings for running a single EKF instance are:
    ```
    EKF2_MULTI_IMU = 0
    EKF2_MULTI_MAG = 0
    SENS_IMU_MODE = 1
    SENS_MAG_MODE = 1
    ```

* logging from boot until disarm: set `SDLOG_MODE` to 1 to log from boot
* Check for no sensor losses or artefacts especially IMU afterwards
* Follow the instructions for system-wide replay on PX4's documentation pages [here](https://docs.px4.io/main/en/debug/system_wide_replay.html#ekf2-replay)