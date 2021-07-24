function [T01 T02 T03] = tmatrixscriptMOD(theta1,theta2,D3)
%units in cm and deg

%Function that returns resultant transformation matrix T03 when given
%joint variables theta1,theta2, and D3.
%---------------------------------------------------------------------%
%inputs for DH parameters
L1 = 0.6;
L2 = 0.4;
L3 = 0.1;
L4 = 0.2;

a1 = L2;
a2 = L4;
a3 = 0;

d1 = L1;
d2 = L3;
d3 = D3;

alpha1 = 0;
alpha2 = 180;
alpha3 = 0;

t1 = theta1; %theta value
t2 = theta2; 
t3 = 0;


%----------------------------------------------------------------------%
%A matrix calculation

A1 = amatrixcalc(a1,d1,alpha1,t1);

A2 = amatrixcalc(a2,d2,alpha2,t2);

A3 = amatrixcalc(a3,d3,alpha3,t3);

%---------------------------------------------------------------------%
%Intermediate Transformation Matrix Calculation
T01 = A1;
T02 = A1*A2;
T03 = A1*A2*A3;

%----------------------------------------------------------------------%
%sub function used to calculate A matrix
function a_matrix_output = amatrixcalc(a,d,alpha,theta)
    a_matrix_output = [cosd(theta),-sind(theta)*cosd(alpha),sind(theta)*sind(alpha),a*cosd(theta);
    sind(theta),cosd(theta)*cosd(alpha),-cosd(theta)*sind(alpha),a*sind(theta);
    0,sind(alpha),cosd(alpha),d;
    0,0,0,1];
end
%-----------------------------------------------------------------------%
end