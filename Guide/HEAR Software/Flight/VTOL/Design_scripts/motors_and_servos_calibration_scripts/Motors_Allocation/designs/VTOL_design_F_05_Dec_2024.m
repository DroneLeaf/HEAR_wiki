% The parameters of the tricopter design with no tilting rotors

%Drone Design:
%  GEOMETRY
%           (M1)CW                       x
%          + S1 Tilt right               |
%               |                        |
%               |                        |
%               |                 y <----+
%   (M3)CW ---- X ---- (M2)CCW          z up
     

K1 = 17; %max thrust of M1 in kgf
K23 = 9.5; %max thrust of M2 and M3, assumed to be identical, in kg
mass = 11.75; %in kg
m2m_length = 0.85;%in m, distance from front motor to back motors
m2m_width = 0.68; %in m, distance between two back motors
CoG = 0.245; %in m, measured from the rear motors