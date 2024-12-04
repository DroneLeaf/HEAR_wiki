% The parameters of the tricopter design with two tilting rear rotors

%Drone Design:
%  GEOMETRY
%           (M1)CW                       x
%          + S1 Tilt right               |
%               |                        |
%               |                        |
%               |                 y <----+
%   (M3)CW ---- X ---- (M2)CCW          z up
     

K1 = 10.5; %max thrust of M1 in kg
K23 = 6.5; %max thrust of M2 and M3, assumed to be identical, in kg
mass = 7.7; %in kg
m2m_length = 0.9;%in m, distance from front motor to back motors
m2m_width = 0.58; %in m, distance between two back motors
CoG = 0.33; %in m, measured from the rear motors