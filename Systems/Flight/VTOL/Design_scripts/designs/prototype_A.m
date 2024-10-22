% The parameters of the tricopter design with two tilting rear rotors

%Drone Design:
%  GEOMETRY
%           (M1)CCW                      x
%               |                        ?
%               |                        |
%               |                 y <----+
%   (M3)CW ---- X ---- (M2)CW          z up
% + S2 Tilt back      + S3 Tilt forward     

K1 = 5.5; %max thrust of M1 in kg
K23 = 4.5; %max thrust of M2 and M3, assumed to be identical, in kg
mass = 5.3; %in kg
m2m_length = 0.86;%in m, distance from front motor to back motors
m2m_width = 0.43; %in m, distance between two back motors
CoG = 0.325; %in m, measured from the rear motors