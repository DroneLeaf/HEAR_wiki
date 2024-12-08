%% All angles are in degrees!
%% S8 settings
% IO settings vtol
s8_io_range_min=0;
s8_io_range_max_vtol=(s8_max_pwm_vtol-s8_min_pwm)/s8_pwm_range;
s8_idle_io=(s8_idle_pwm-s8_min_pwm_absolute)/s8_pwm_range;

% Allocation vtol
allocation_range_s8=s8_io_range_max_vtol;
s8_alloc_min=-s8_idle_io;
s8_alloc_max=s8_io_range_max_vtol-s8_idle_io;

s8_alloc_gain_pos=s8_alloc_max;
s8_alloc_gain_neg=abs(s8_alloc_min);

% IO settings plane
s8_io_range_min_plane=1-(s8_max_pwm_plane-s8_min_pwm_plane)/s8_pwm_range;
s8_io_range_max_plane=1;
s8_idle_io_plane=1-(s8_pwm_at_90deg-s8_min_pwm_plane)/s8_pwm_range;

% Allocation plane
s8_alloc_min_plane=s8_io_range_min_plane-s8_idle_io_plane;
s8_alloc_max_plane=s8_io_range_max_plane-s8_idle_io_plane;

s8_alloc_gain_pos_plane=s8_alloc_max_plane;
s8_alloc_gain_neg_plane=abs(s8_alloc_min_plane);