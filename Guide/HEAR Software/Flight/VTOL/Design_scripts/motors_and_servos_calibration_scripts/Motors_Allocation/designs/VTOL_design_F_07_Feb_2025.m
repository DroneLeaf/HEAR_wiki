% The parameters of the tricopter design with no tilting rotors

%Drone Design:
%  GEOMETRY
%           (M1)CW                       x
%          + S1 Tilt right               |
%               |                        |
%               |                        |
%               |                 y <----+
%   (M3)CW ---- X ---- (M2)CCW          z up
     

K1 = 13.2*0.9; %max thrust of M1 in kgf, 20x10CF; with 10perc discount
K23 = 12.2*0.9; %max thrust of M2 and M3, assumed to be identical, in kg, 17x9x3; with 10perc discount
mass = 14.3; %in kg
m2m_length = 0.88;%in m, distance from front motor to back motors
m2m_width = 0.68; %in m, distance between two back motors
CoG = 0.285; %in m, measured from the rear motors