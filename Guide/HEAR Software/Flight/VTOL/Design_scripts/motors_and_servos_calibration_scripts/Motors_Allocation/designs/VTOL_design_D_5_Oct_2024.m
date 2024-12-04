% The parameters of the tricopter design with no tilting rotors

%Drone Design:
%  GEOMETRY
%           (M1)CW                       x
%          + S1 Tilt right               |
%               |                        |
%               |                        |
%               |                 y <----+
%   (M3)CW ---- X ---- (M2)CCW          z up
     

K1 = 11; %max thrust of M1 in kgf
K23 = 11; %max thrust of M2 and M3, assumed to be identical, in kg
mass = 11.11; %in kg
m2m_length = 0.847;%in m, distance from front motor to back motors
m2m_width = 0.68; %in m, distance between two back motors
CoG = 0.283; %in m, measured from the rear motors