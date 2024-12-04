%% All angles are in degrees!
%% S8 settings
% IO settings
s8_io_range_min=0;
s8_io_range_max=(s8_max_pwm_vtol-s8_min_pwm)/s8_pwm_range;
s8_idle_io=(s8_idle_pwm-s8_min_pwm)/s8_pwm_range;

% Allocation settings
allocation_range_s8=s8_io_range_max;
s8_alloc_min=-s8_idle_io;
s8_alloc_max=s8_io_range_max-s8_idle_io;

s8_alloc_gain_pos=s8_alloc_max;
s8_alloc_gain_neg=abs(s8_alloc_min);


