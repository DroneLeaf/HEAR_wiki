%% All angles are in degrees!
%% S13 settings
% IO settings
s13_io_range_min=0;
s13_io_range_max=1;
s13_idle_io=(s13_idle_pwm-s13_min_pwm)/s13_pwm_range;

% Allocation settings
s13_alloc_min=-((s13_idle_pwm-s13_min_pwm)/s13_pwm_range);
s13_alloc_max=((s13_max_pwm-s13_idle_pwm)/s13_pwm_range);

s13_alloc_gain_pos=(s13_io_range_max-s13_idle_io);
s13_alloc_gain_neg=s13_idle_io;
% Assuming forward input range is from -1 to 1

s13_alloc_gain_pos_fwd_slider=s13_alloc_gain_neg;
s13_alloc_post_alloc_bias=s13_alloc_min;
%% S14 settings
% IO settings
s14_io_range_min=0;
s14_io_range_max=1;
s14_idle_io=(s14_idle_pwm-s14_min_pwm)/s14_pwm_range;

% Allocation settings
s14_alloc_min=-((s14_idle_pwm-s14_min_pwm)/s14_pwm_range);
s14_alloc_max=((s14_max_pwm-s14_idle_pwm)/s14_pwm_range);

s14_alloc_gain_pos=(s14_io_range_max-s14_idle_io);
s14_alloc_gain_neg=s14_idle_io;

s14_alloc_gain_pos_fwd_slider=s14_alloc_gain_neg;
s14_alloc_post_alloc_bias=s14_alloc_min;