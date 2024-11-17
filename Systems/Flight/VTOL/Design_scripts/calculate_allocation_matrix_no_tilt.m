% This script calculates the allocation matrix for a tricopter with two rear tilting rotors.
% The user can change the maximum thrust of each motor, the UAV mass, Center of Gravitiy (CoG), the length, and width of the UAV
% The script outputs the allocation matrix, along with a column-normalized allocation matrix for better interpretability
% The script also calculates the maximum effective roll, and pitch moments,and the maximum effective thrust.


clear all
close all

%% 
% How to run
% Step A: Create a .M file under the designs directory that sets the design parameters of your drone,
% see designs\prototype_A.m as an example

% Step B: Chance the design_name variable below to the filename of the .m
% file you created

% Step C: Run the script, it will automatically output the max effective
% thrust and moments, the allocation matrix, and a normalized version of
% the allocation matrix

design_name = 'VTOL_design_F_11_Nov_2024';

%%
% Motor Convention:
% tri_rear_tilt
%  GEOMETRY
%           (M1)CCW                      x
%               |                        ?
%               |                        |
%               |                 y <----+
%   (M3)CW ---- X ---- (M2)CW          z up
% + S2 Tilt back      + S3 Tilt forward                         
%                                      
% 
%  For Positive Pitch, all motors with positive x should be decreases
%  For Negative Roll, all motors with negative Y should be increased
%  For Positive Yaw, all servo tilt should be increased
%  POSITIVE PITCH result in moving in the direction of POSITIVE X
%  NEGATIVE ROLL result in moving in the direction of POSITIVE Y 
%%
%STEP A: Calculate moment matrix and allocation matrix sybolically

% [roll_u pitch_u yaw_u thrust]' = moment_matrix * [M1 M2 M3 S1 S2]'

syms K1 K23 X1 X23 Y23 M L CoG
%K1:  max thrust of M1 in kg
%K23: max thrust of M2 or M3, assumed to be identical, in kg
%X1:  The absolute value of the distance from CoG to M1 along drone x-axis (front), in m
%X23:  The absolute value of the distance from CoG to M2 or M3, assumed to be identical, along drone x-axis (front), in m
%Y23:  The absolute value of the distance from CoG to M2 or M3, assumed to be identical, along drone y-axis (front), in m
%M:  The mass of the UAV, in kg
%L: The distance between the two read motors and the front motor, in m
%CoG: The CoG of the drone measured from the rear motors, in m


moment_matrix_sym = [   0               -9.81*Y23*K23   9.81*Y23*K23    0           0;
                        -9.81*K1*X1     9.81*K23*X23    9.81*K23*X23    0           0;
                        0               0               0               sqrt(0.5)   sqrt(0.5);
                        K1              K23             K23             0           0];
               
allocation_matrix_sym = pinv(moment_matrix_sym);


%%
%STEP B: load parameters based on drone design and evaluate
%allocation matrix

run(strcat('designs/',design_name, '.m')); % run the script to load the parameters

X1 = m2m_length - CoG;
X23 = CoG;
Y23 = m2m_width / 2;

allocation_matrix = double(subs(allocation_matrix_sym))


%%
%STEP C: Evaluate max effective thrust

%calculate nominal thrust command
motors_at_unit_thrust = allocation_matrix(:,4);
motors_at_max_thrust = motors_at_unit_thrust / max(motors_at_unit_thrust);

moment_matrix = double(subs(moment_matrix_sym));
max_effective_thrust = moment_matrix(4,:) * motors_at_max_thrust %in kg

effective_thrust_2_weight = max_effective_thrust / mass

%%
%STEP D: Evaluate max possible pitch moment in m based on design

moment_matrix = double(subs(moment_matrix_sym));

minimum_pitch_moment = moment_matrix(2,:) * [1 0 0 0 0]'
maximum_pitch_moment = moment_matrix(2,:) * [0 1 1 0 0]'

max_absolute_pitch_moment = min([maximum_pitch_moment, abs(minimum_pitch_moment)])

%%
%STEP E: Evaluate max possible roll moment in Nm based on design

moment_matrix = double(subs(moment_matrix_sym));

max_absolute_roll_moment =  moment_matrix(2,:) * [0 1 0 0 0]'

%%
%STEP F: Out
% Normalize each column based on max value

%scale thrust, roll, and pitch columns based on max value

allocation_matrix_normalized = allocation_matrix;

%scale thrust column
allocation_matrix_normalized(:,4) = allocation_matrix_normalized(:,4) / max(abs(allocation_matrix_normalized(:,4)));

%scale roll column
allocation_matrix_normalized(:,1) = allocation_matrix_normalized(:,1) * 0.7071 / max(abs(allocation_matrix_normalized(:,1)));

%scale pitch column
allocation_matrix_normalized(:,2) = allocation_matrix_normalized(:,2) * 0.7071 / max(abs(allocation_matrix_normalized(:,2)));

allocation_matrix_normalized

