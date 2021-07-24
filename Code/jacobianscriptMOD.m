function [JVO1, JVO2, JVO3] = jacobianscriptMOD(theta1,theta2,D3)
%function that takes joint variables(theta1, theta2, d3) input and outputs 
%jacobian matrix

%-------------------------------------------------------------------------%
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
T1 = A1;
T2 = T1*A2;
T3 = T2*A3;

%---------------------------------------------------------------------%
%Calculating Parameters needed for Jacobian Matrix
%(Z0,Z1,Z2,Z3,Z4,Z5,O0,O1,O2,O3,O4,O5,O6)

%extracting z column vectors from respective Tmatrix
Z0 = [0;0;1];
Z1 = T1(1:3,3);
Z2 = T2(1:3,3);

%extracting O vectors from respective Tmatrix
O0 = [0;0;0];
O1 = T1(1:3,4);
O2 = T2(1:3,4);
O3 = T3(1:3,4);

OHAT = [0;0;0];

%---------------------------------------------------------------------%
%This section from HW2 not used in HW3
%Calculate J1,J2,J3
% J1 = [cross(Z0,O3-O0);Z0];
% J2 = [cross(Z1,O3-O1);Z1];
% J3 = [Z2;OHAT];
    
%---------------------------------------------------------------------%
%calculate Jacobian matrices for each frame

JVO1 = [cross(Z0,O1-O0),OHAT,OHAT];
JVO2 = [cross(Z0,O2-O0),cross(Z1,O2-O1),OHAT];
JVO3 = [cross(Z0,O3-O0),cross(Z1,O3-O1),Z2];
%-------------------------------------------------------------------------%
%sub function used to calculate A matrix
function a_matrix_output = amatrixcalc(a,d,alpha,theta)
    a_matrix_output = [cosd(theta),-sind(theta)*cosd(alpha),sind(theta)*sind(alpha),a*cosd(theta);
    sind(theta),cosd(theta)*cosd(alpha),-cosd(theta)*sind(alpha),a*sind(theta);
    0,sind(alpha),cosd(alpha),d;
    0,0,0,1];
end

%-----------------------------------------------------------------------%
end