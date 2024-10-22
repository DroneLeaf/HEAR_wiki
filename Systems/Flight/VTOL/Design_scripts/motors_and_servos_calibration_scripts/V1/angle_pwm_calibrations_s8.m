%% All angles are in degrees!
clear all
%% S6 settings
% Physical settings
front_tilt_servo_s8_min_pwm=1100;
front_tilt_servo_s8_max_pwm=1800;
front_tilt_servo_s8_center_pwm=1300;

front_tilt_servo_s8_angle_at_min_pwm=-7;
front_tilt_servo_s8_angle_at_max_pwm=30;
% Assuming s8 center angle is zero

% HEAR_Configurations
minimum_actuator_cmds_s8=0.0;
maximum_actuator_cmds_s8=1.0;
allocation_range_s8=maximum_actuator_cmds_s8-minimum_actuator_cmds_s8;

servo_s8_pwm_range=front_tilt_servo_s8_max_pwm-front_tilt_servo_s8_min_pwm;
servo_s8_post_allocation_bias=(front_tilt_servo_s8_center_pwm-front_tilt_servo_s8_min_pwm)/servo_s8_pwm_range

forward_to_servo_s8_alloc_pos_gain=allocation_range_s8*((front_tilt_servo_s8_max_pwm-front_tilt_servo_s8_center_pwm)/servo_s8_pwm_range)
forward_to_servo_s8_alloc_neg_gain=allocation_range_s8*((front_tilt_servo_s8_center_pwm-front_tilt_servo_s8_min_pwm)/servo_s8_pwm_range)

