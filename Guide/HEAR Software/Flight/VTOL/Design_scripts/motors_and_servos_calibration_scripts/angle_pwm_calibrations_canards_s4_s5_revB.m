%% All angles are in degrees!
%% S4 settings - VTOL mode
% Set IO range
s4_io_range_min=0;
s4_io_range_max=1;
s4_io_range_min_plane=(s4_min_pwm_plane-s4_min_pwm)/s4_pwm_range;
s4_io_range_max_plane=s4_io_range_max+(s4_max_pwm_plane-s4_max_pwm)/s4_pwm_range;
s4_io_idle=(s4_idle_pwm-s4_min_pwm)/s4_pwm_range;

% Set Allocation range
s4_alloc_min=s4_io_range_min-s4_io_idle;
s4_alloc_max=s4_io_range_max+s4_alloc_min;
s4_alloc_pos_gain=s4_alloc_max;
s4_alloc_neg_gain=abs(s4_alloc_min);

s4_alloc_min_plane=s4_io_range_min_plane-s4_io_idle;
s4_alloc_max_plane=s4_io_range_max_plane-s4_io_idle;
s4_alloc_pos_gain_plane=s4_alloc_max_plane;
s4_alloc_neg_gain_plane=abs(s4_alloc_min_plane);

%% S5 settings - VTOL mode
% % Set IO range
% s5_io_range_min=0;
% s5_io_range_max=1;
% s5_io_range_min_plane=(s5_min_pwm_plane-s5_min_pwm)/s5_pwm_range;
% s5_io_range_max_plane=s5_io_range_max+(s5_max_pwm_plane-s5_max_pwm)/s5_pwm_range;
% s5_io_idle=(s5_idle_pwm-s5_min_pwm)/s5_pwm_range;
% 
% % Set Allocation range
% s5_alloc_min=s5_io_range_min-s5_io_idle;
% s5_alloc_max=s5_io_range_max+s5_alloc_min;
% s5_alloc_pos_gain=s5_alloc_max;
% s5_alloc_neg_gain=abs(s5_alloc_min);
% 
% s5_alloc_min_plane=s5_io_range_min_plane-s5_io_idle;
% s5_alloc_max_plane=s5_io_range_max_plane-s5_io_idle;
% s5_alloc_pos_gain_plane=s5_alloc_max_plane;
% s5_alloc_neg_gain_plane=abs(s5_alloc_min_plane);