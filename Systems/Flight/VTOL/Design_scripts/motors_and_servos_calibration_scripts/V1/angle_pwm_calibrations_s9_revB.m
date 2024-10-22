%% All angles are in degrees!
%% S9 settings
% IO settings
s9_io_range_min=0;
s9_io_range_max=1;
s9_idle_io=(s9_idle_pwm-s9_min_pwm)/s9_pwm_range;

% Allocation settings
s9_min_alloc=-((s9_idle_pwm-s9_min_pwm)/s9_pwm_range);
s9_max_alloc=((s9_max_pwm-s9_idle_pwm)/s9_pwm_range);

s9_alloc_gain_pos=(s9_io_range_max-s9_idle_io)*2;
s9_alloc_gain_neg=s9_idle_io*2;
% Assuming forward input range is from -1 to 1
% forward_to_servo_s9_alloc_pos_gain=maximum_actuator_cmds_s9
% forward_to_servo_s9_alloc_neg_gain=-minimum_actuator_cmds_s9

%% S10 settings
% IO settings
s10_io_range_min=0;
s10_io_range_max=1;
s10_idle_io=(s10_idle_pwm-s10_min_pwm)/s10_pwm_range;

% Allocation settings
s10_min_alloc=-((s10_idle_pwm-s10_min_pwm)/s10_pwm_range);
s10_max_alloc=((s10_max_pwm-s10_idle_pwm)/s10_pwm_range);

s10_alloc_gain_pos=(s10_io_range_max-s10_idle_io)*2;
s10_alloc_gain_neg=s10_idle_io*2;
% Assuming forward input range is from -1 to 1
% forward_to_servo_s10_alloc_pos_gain=maximum_actuator_cmds_s9
% forward_to_servo_s9_alloc_neg_gain=-minimum_actuator_cmds_s9