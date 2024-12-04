%% All angles are in degrees!
%% S6 settings - VTOL mode
% Set IO range
s6_io_range_min=0;
s6_io_range_max=1+(s6_max_pwm_vtol-s6_max_pwm)/s6_pwm_range;
s6_io_range_max_plane=1;
s6_io_idle=(s6_idle_pwm-s6_min_pwm)/s6_pwm_range;

% Assuming s6 center angle is zero
% s6_angle_at_min_pwm=-30;
% s6_angle_at_max_pwm=30;
% Set Allocation range
% s6_alloc_min=deg2rad(s6_angle_at_min_pwm);
% s6_alloc_max=deg2rad(s6_angle_at_max_pwm);
% s6_alloc_pos_gain=(s6_max_pwm_vtol-s6_idle_pwm)/s6_pwm_range_vtol;
% s6_alloc_neg_gain=(s6_idle_pwm-s6_min_pwm)/s6_pwm_range_vtol;


s6_alloc_min=s6_io_range_min-s6_io_idle;
s6_alloc_max=s6_io_range_max+s6_alloc_min;
s6_alloc_pos_gain=s6_alloc_max;
s6_alloc_neg_gain=abs(s6_alloc_min);

s6_alloc_min_plane=s6_io_range_min-s6_io_idle;
s6_alloc_max_plane=s6_io_range_max_plane-s6_io_idle;
s6_alloc_pos_gain_plane=s6_alloc_max_plane;
s6_alloc_neg_gain_plane=abs(s6_alloc_min_plane);

% HEAR_Configurations
% servo_s6_post_allocation_bias_a0=(right_vane_servo_s6_center_pwm-right_vane_servo_s6_min_pwm_vtol)/right_vane_servo_s6_pwm_range_vtol;
% servo_s6_post_allocation_bias=(servo_s6_post_allocation_bias_a0*allocation_range_s6)+minimum_actuator_cmds_s6
% 
% forward_to_servo_s6_alloc_pos_gain=allocation_range_s6*((right_vane_servo_s6_max_pwm-right_vane_servo_s6_center_pwm)/right_vane_servo_s6_pwm_range_vtol)
% forward_to_servo_s6_alloc_neg_gain=allocation_range_s6*((right_vane_servo_s6_center_pwm-right_vane_servo_s6_min_pwm_vtol)/right_vane_servo_s6_pwm_range_vtol)

%% S7 settings - VTOL mode
% Set IO range
s7_io_range_min=(s7_min_pwm_vtol-s7_min_pwm)/s7_pwm_range;
s7_io_range_min_plane=0;
s7_io_range_max=1;
s7_io_idle=(s7_idle_pwm-s7_min_pwm)/s7_pwm_range;

% 
s7_alloc_min=-(s7_io_idle-s7_io_range_min);
s7_alloc_max=s7_io_range_max-s7_io_idle;
s7_alloc_pos_gain=s7_alloc_max;
s7_alloc_neg_gain=abs(s7_alloc_min);

s7_alloc_min_plane=-s7_io_idle;
s7_alloc_max_plane=s7_io_range_max-s7_io_idle;
s7_alloc_pos_gain_plane=s7_alloc_max_plane;
s7_alloc_neg_gain_plane=abs(s7_alloc_min_plane);


% HEAR_Configurations
% minimum_actuator_cmds_s7=deg2rad(right_vane_servo_s7_angle_at_min_pwm_vtol)
% maximum_actuator_cmds_s7=deg2rad(right_vane_servo_s7_angle_at_max_pwm)
% allocation_range_s7=maximum_actuator_cmds_s7-minimum_actuator_cmds_s7;
% 
% servo_s7_post_allocation_bias_a0=(right_vane_servo_s7_center_pwm-right_vane_servo_s7_min_pwm_vtol)/right_vane_servo_s7_pwm_range_vtol;
% servo_s7_post_allocation_bias=(servo_s7_post_allocation_bias_a0*allocation_range_s7)+minimum_actuator_cmds_s7
% 
% forward_to_servo_s7_alloc_pos_gain=allocation_range_s7*((right_vane_servo_s7_max_pwm-right_vane_servo_s7_center_pwm)/right_vane_servo_s7_pwm_range_vtol)
% forward_to_servo_s7_alloc_neg_gain=allocation_range_s7*((right_vane_servo_s7_center_pwm-right_vane_servo_s7_min_pwm_vtol)/right_vane_servo_s7_pwm_range_vtol)