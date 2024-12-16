# How to setup
In the instance set "enable_rc_pos_setpoint": true

Make sure "en_receive_raw_rc_over_ros": false in UAV_types/[Type_name]/px4_rc_to_ori_thrust/general.json

Set the desired control behaviour in UAV_types/[Type_name]/rc_to_pos_vel_yaw_set_points

## Setting Cuboid limits
Example: rc_pos_x_hard_limits sets the hard limit, i.e. set point does not exceed these limits.

rc_pos_x_soft_limits the drone starts to slow down at these limits