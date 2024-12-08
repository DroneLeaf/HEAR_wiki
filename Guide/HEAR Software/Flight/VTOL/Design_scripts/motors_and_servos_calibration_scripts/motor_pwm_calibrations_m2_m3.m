%% All angles are in degrees!
%% M2 settings - VTOL mode
% Set IO range
m2_io_range_min=1-(m2_pwm_range_spining/m2_pwm_range);
m2_io_range_max=1;
m2_io_range_min_plane=0;
m2_io_range_max_plane=1;
m2_io_idle=m2_io_range_min;

%% M3 settings - VTOL mode
% Set IO range
m3_io_range_min=1-(m3_pwm_range_spining/m3_pwm_range);
m3_io_range_max=1;
m3_io_range_min_plane=0;
m3_io_range_max_plane=1;
m3_io_idle=m3_io_range_min;