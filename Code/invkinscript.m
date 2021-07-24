function [Sol1 Sol2] = invkinscript(givenx,giveny,givenz)
%function that takes the end effector position as input(x,y,z) and outputs
%two configuration vectors, the first being the elbow up configuration and 
%the second the elbow down configuration

%rounding to 10 decimal places to prevent issues with truncation errors in
%matlab
x = round(givenx,10);
y = round(giveny,10);
z = round(givenz,10);
%-------------------------------------------------------------------------%
%Link Lengths for calculation
L1 = 0.6;
L2 = 0.4;
L3 = 0.1;
L4 = 0.2;

%-------------------------------------------------------------------------%
%calculating joint variable d3
d3 = L1 + L3 - z;

%-------------------------------------------------------------------------%
%calculating theta2
C2 = ((x^2) + (y^2) - ((L2)^2) - ((L4)^2))/(2*(L2)*(L4));

%calculating both positive and negative case for S2
S2_pos = sqrt(1-((C2)^2));
S2_neg = -S2_pos;
theta2_pos = atan2d(S2_pos,C2);
theta2_neg = atan2d(S2_neg,C2);

%-------------------------------------------------------------------------%
%calculating theta1 for each case of theta2

alpha = atan2d(y,x);
beta = acosd((((L2)^2)+ (x^2) + (y^2) - (L4)^2)/(2*(L2)*sqrt((x^2)+(y^2))));

%for case of theta2_pos calculate respective theta1_pos
    theta1_pos = alpha - beta;
%map theta1_pos to be between 0 and 360
    theta1_pos = theta1_pos + 360*(theta1_pos < 0);
%for case of theta2_pos calculate respective theta1_pos
    theta1_neg = alpha + beta;

%output the two configuration vectors
Sol1 = [theta1_pos,theta2_pos,d3];
Sol2 = [theta1_neg,theta2_neg,d3];
    
end

