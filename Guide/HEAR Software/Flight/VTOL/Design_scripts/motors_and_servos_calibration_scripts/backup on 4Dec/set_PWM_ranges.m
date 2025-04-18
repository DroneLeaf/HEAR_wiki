%% Info
% Revision: 24-Nov-2024
% Data Entry: Mohamad Chehadeh
% Copyright for DroneLeaf LLC
%% Physical settings
%M1, M2, M3
m1_min_pwm=1050;
m1_max_pwm=1920;
m2_min_pwm=1000;
m2_min_pwm_spining=1050;
m2_max_pwm=1920;
m3_min_pwm=1000;
m3_min_pwm_spining=1050;
m3_max_pwm=1920;

m2_pwm_range=m2_max_pwm-m2_min_pwm;
m2_pwm_range_spining=m2_max_pwm-m2_min_pwm_spining;

m3_pwm_range=m3_max_pwm-m3_min_pwm;
m3_pwm_range_spining=m3_max_pwm-m3_min_pwm_spining;


%S4
s4_idle_pwm_rev=1500;
s4_min_pwm_rev=1330;
s4_max_pwm_rev=1670;
s4_pwm_range_high_rev=s4_max_pwm_rev-s4_idle_pwm_rev;

s4_idle_pwm=s4_pwm_range_high_rev+s4_min_pwm_rev;
s4_min_pwm=s4_min_pwm_rev;
s4_max_pwm=s4_max_pwm_rev;

s4_angle_at_min_pwm=-15;
s4_angle_at_max_pwm=15;
s4_pwm_range=s4_max_pwm-s4_min_pwm;

s4_min_pwm_plane=1340;
s4_max_pwm_plane=1640;
s4_idle_pwm_plane=1500;
s4_angle_at_min_pwm_plane=-10;
s4_angle_at_max_pwm_plane=10;
s4_pwm_range_plane=s4_max_pwm_plane-s4_min_pwm_plane;

%S5

% s5_min_pwm=1200;
% s5_max_pwm=1840;
% s5_idle_pwm=1510;
% s5_angle_at_min_pwm=-30;
% s5_angle_at_max_pwm=30;
% s5_pwm_range=s5_max_pwm-s5_min_pwm;
% 
% s5_min_pwm_plane=1410;
% s5_max_pwm_plane=1620;
% s5_idle_pwm_plane=1510;
% s5_angle_at_min_pwm_plane=-10;
% s5_angle_at_max_pwm_plane=10;
% s5_pwm_range_plane=s5_max_pwm_plane-s5_min_pwm_plane;

%S6
s6_idle_pwm=1180;
s6_min_pwm=720;
s6_max_pwm=2160;
s6_max_pwm_vtol=1640;
s6_pwm_range=s6_max_pwm-s6_min_pwm;
s6_pwm_range_vtol=s6_max_pwm_vtol-s6_min_pwm;

s6_angle_at_min_pwm=-40;
s6_angle_at_max_pwm=40;

s6_angle_at_min_fwd_cmd=-15;
s6_angle_at_max_fwd_cmd=15;

% Calculate with linear assumption
s6_pwm_at_min_fwd_cmd=s6_idle_pwm-(s6_idle_pwm-s6_min_pwm)*(s6_angle_at_min_fwd_cmd/s6_angle_at_min_pwm);
s6_pwm_at_max_fwd_cmd=s6_idle_pwm+(s6_max_pwm_vtol-s6_idle_pwm)*(s6_angle_at_max_fwd_cmd/s6_angle_at_max_pwm);

%S7
s7_idle_pwm=1820;
s7_min_pwm=840;
s7_max_pwm=2280;
s7_min_pwm_vtol=1360;
s7_pwm_range=s7_max_pwm-s7_min_pwm;
s7_pwm_range_vtol=s7_max_pwm-s7_min_pwm_vtol;

s7_angle_at_min_pwm=-40;
s7_angle_at_max_pwm=40;

s7_angle_at_min_fwd_cmd=-15;
s7_angle_at_max_fwd_cmd=15;

s7_pwm_at_min_fwd_cmd=s7_idle_pwm-round((s7_idle_pwm-s7_min_pwm_vtol)*(s7_angle_at_min_fwd_cmd/s7_angle_at_min_pwm));
s7_pwm_at_max_fwd_cmd=s7_idle_pwm+round((s7_max_pwm-s7_idle_pwm)*(s7_angle_at_max_fwd_cmd/s7_angle_at_max_pwm));

%S8
s8_idle_pwm_rev=2000;
s8_min_pwm_rev=1480;
s8_max_pwm_rev=2075;
s8_pwm_range_high_rev=s8_max_pwm_rev-s8_idle_pwm_rev;

s8_idle_pwm=s8_pwm_range_high_rev+s8_min_pwm_rev;
s8_min_pwm=s8_min_pwm_rev;
s8_max_pwm=s8_max_pwm_rev;
s8_pwm_range=s8_max_pwm-s8_min_pwm;
s8_angle_at_min_pwm=-10;
s8_angle_at_max_pwm=60;
s8_angle_at_max_pwm_vtol=60; 
s8_max_pwm_vtol=s8_idle_pwm+(s8_max_pwm-s8_idle_pwm)*(s8_angle_at_max_pwm_vtol/s8_angle_at_max_pwm);% Linearity assumed
s8_pwm_range_vtol=s8_max_pwm_vtol-s8_min_pwm;


%S9
s9_idle_pwm=1500;
s9_min_pwm=1000;
s9_max_pwm=2000;
s9_pwm_range=s9_max_pwm-s9_min_pwm;
s9_min_range_perc=(s9_idle_pwm-s9_min_pwm)/s9_pwm_range;
s9_max_range_perc=(s9_max_pwm-s9_idle_pwm)/s9_pwm_range;

% %S10
% s10_idle_pwm=1500;
% s10_min_pwm=1220;
% s10_max_pwm=1780;
% s10_pwm_range=s10_max_pwm-s10_min_pwm;
% s10_min_range_perc=(s10_idle_pwm-s10_min_pwm)/s10_pwm_range;
% s10_max_range_perc=(s10_max_pwm-s10_idle_pwm)/s10_pwm_range;

%S11 AND S12 by RC

%S13
s13_idle_pwm_rev=1485;
s13_min_pwm_rev=1020;
s13_max_pwm_rev=1930;
s13_pwm_range_rev=s13_max_pwm_rev-s13_min_pwm_rev;
s13_pwm_range_high_rev=s13_max_pwm_rev-s13_idle_pwm_rev;

s13_idle_pwm=s13_pwm_range_high_rev+s13_min_pwm_rev;
s13_min_pwm=s13_min_pwm_rev;
s13_max_pwm=s13_max_pwm_rev;
s13_pwm_range=s13_max_pwm-s13_min_pwm;
s13_angle_at_min_pwm_vtol=-40;
s13_angle_at_max_pwm_vtol=40;

%S14
s14_idle_pwm=1485;
s14_min_pwm=1080;
s14_max_pwm=1900;
s14_pwm_range=s14_max_pwm-s14_min_pwm;
s14_angle_at_min_pwm_vtol=-40;
s14_angle_at_max_pwm_vtol=40;

%S15

%S16
s16_idle_pwm=1000;
s16_min_pwm=1000;
s16_max_pwm=2000;
s16_pwm_range=s16_max_pwm-s16_min_pwm;
s16_is_reversed=-1;
%S17 and S18
s17_idle_pwm=2000;
s17_min_pwm=1000;
s17_max_pwm=2000;
s17_pwm_range=s17_max_pwm-s17_min_pwm;

%S19
s19_idle_pwm=2000;
s19_min_pwm=1000;
s19_max_pwm=2000;
s19_pwm_range=s19_max_pwm-s19_min_pwm;