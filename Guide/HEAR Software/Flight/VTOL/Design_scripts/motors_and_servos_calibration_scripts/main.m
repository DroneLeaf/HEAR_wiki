%% Set based on measurements
clear all
clc
addpath(genpath(pwd+"/Motors_Allocation"));
%% Input calibration values
set_PWM_ranges;

%% Calculate servo limits
angle_pwm_calibrations_canards_s4_s5_revB;
angle_pwm_calibrations_vanes_s6_s7_revC;
angle_pwm_calibrations_s8_revB;
angle_pwm_calibrations_s9_revB;
angle_pwm_calibrations_s13_s14;

%% Calculate motors allocation
% Note: run this segment from the 'V1'
design_name = 'VTOL_design_F_26_Nov_2024';
calculate_allocation_matrix_no_tilt;

%% Generate configurations files
generate_hear_configurations_parameters