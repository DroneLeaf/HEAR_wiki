%% All angles are in degrees!
%% S6 settings - VTOL mode
% Set IO range
s6_io_range_min=0;
s6_io_range_max=1+(s6_max_pwm_vtol-s6_max_pwm)/s6_pwm_range;
s6_io_range_max_plane=1;
s6_io_idle=(s6_idle_pwm-s6_min_pwm)/s6_pwm_range;

s6_alloc_min=s6_io_range_min-s6_io_idle;
s6_alloc_max=s6_io_range_max+s6_alloc_min;
s6_alloc_pos_gain=s6_alloc_max;
s6_alloc_neg_gain=abs(s6_alloc_min);

s6_alloc_min_plane=s6_io_range_min-s6_io_idle;
s6_alloc_max_plane=s6_io_range_max_plane-s6_io_idle;
s6_alloc_pos_gain_plane=s6_alloc_max_plane;
s6_alloc_neg_gain_plane=abs(s6_alloc_min_plane);

% these io ranges are auxillary, just for subsequent calculations
s6_io_range_min_for_fwd_cmd=(s6_pwm_at_min_fwd_cmd-s6_min_pwm)/s6_pwm_range;
s6_io_range_max_for_fwd_cmd=1+(s6_pwm_at_max_fwd_cmd-s6_max_pwm)/s6_pwm_range;

s6_alloc_pos_gain_fwd_cmd=((s6_io_range_max_for_fwd_cmd-s6_io_idle)/(s6_io_range_max-s6_io_idle))*s6_alloc_pos_gain;
s6_alloc_neg_gain_fwd_cmd=((s6_io_idle-s6_io_range_min_for_fwd_cmd)/(s6_io_idle-s6_io_range_min))*s6_alloc_neg_gain;

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

% these io ranges are auxillary, just for subsequent calculations
s7_io_range_min_for_fwd_cmd=(s7_pwm_at_min_fwd_cmd-s7_min_pwm)/s7_pwm_range;
s7_io_range_max_for_fwd_cmd=1+(s7_pwm_at_max_fwd_cmd-s7_max_pwm)/s7_pwm_range;

s7_alloc_pos_gain_fwd_cmd=((s7_io_range_max_for_fwd_cmd-s7_io_idle)/(s7_io_range_max-s7_io_idle))*s7_alloc_pos_gain;
s7_alloc_neg_gain_fwd_cmd=((s7_io_idle-s7_io_range_min_for_fwd_cmd)/(s7_io_idle-s7_io_range_min))*s7_alloc_neg_gain;