function [FrepO1, FrepO2, FrepO3] = repulsive(CurrQ,aeta1,aeta2,aeta3)
%function that takes current configuration and aeta values for O1,O2,O3
%then computes the repulisive forces of applicable obstacles on O1,O2,O3
%returns the repulsive force for O1,O2,O3
%aeta1 for O1
%aeta2 for O2
%aeta3 for O3
%Determine joint origins in current configuration
[T01C, T02C, T03C] = tmatrixscriptMOD(CurrQ(1),CurrQ(2),CurrQ(3));

O1C = T01C(1:3,4);
O2C = T02C(1:3,4);
O3C = T03C(1:3,4);
%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%
%repulsive forces for left conveyor rail (LR) (Quadrant 2,3)

%O1
row0LR = 0.15; %region of influence for Left Rail

%treating rail as line segment, finding shortest distance to rail
PLR = [-0.38;0.7;0.275];
QLR = [-0.38;-0.8;0.275];

%Finding nearest point on line segment
dist = dot((PLR-QLR),(QLR-O1C))/dot((PLR-QLR),(PLR-QLR));
NP = QLR - dist*(PLR-QLR); %Nearest Point

%calculate distance of O1 to nearest point on left rail line segment
RowOfO1CLR = norm(O1C - NP);

%determine whether O1 falls in the region of influnce or not
if RowOfO1CLR <= row0LR
    GradOfO1CLR = (O1C - NP)/(norm(O1C-NP));
    FrepLeftRailO1 = aeta1*((1/RowOfO1CLR) - (1/row0LR))*(1/RowOfO1CLR^2)*GradOfO1CLR;
else
    %if outside of region of influence set force to 0
    FrepLeftRailO1 = [0;0;0]; 
end

%-------------------------------------------------------------------------%
%Repeat calculation for repulsive force for O2

%Finding nearest point on line segment
dist = dot((PLR-QLR),(QLR-O2C))/dot((PLR-QLR),(PLR-QLR));
NP = QLR - dist*(PLR-QLR);

%calculate distance of O2 to nearest point on left rail line segment
RowOfO2CLR = norm(O2C - NP);

%determine whether O2 falls in the region of influnce or not
if RowOfO2CLR <= row0LR
    GradOfO2CLR = (O2C - NP)/(norm(O2C-NP));
    FrepLeftRailO2 = aeta1*((1/RowOfO2CLR) - (1/row0LR))*(1/(RowOfO2CLR^2))*GradOfO2CLR;
else
    %if outside of region of influence set force to 0
    FrepLeftRailO2 = [0;0;0]; 
end

%-------------------------------------------------------------------------%
%Repeat calculation for repulsive force for O3

%Finding nearest point on line segment
dist = dot((PLR-QLR),(QLR-O3C))/dot((PLR-QLR),(PLR-QLR));
NP = QLR - dist*(PLR-QLR);

%calculate distance of O3 to nearest point on left rail line segment
RowOfO3CLR = norm(O3C - NP);

%determine whether O3 falls in the region of influnce or not
if RowOfO3CLR <= row0LR
    GradOfO3CLR = (O3C - NP)/(norm(O3C-NP));
    FrepLeftRailO3 = aeta1*((1/RowOfO3CLR) - (1/row0LR))*(1/(RowOfO3CLR^2))*GradOfO3CLR;
else
    %if outside of region of influence set force to 0
    FrepLeftRailO3 = [0;0;0]; 
end

%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%
%Determine Repulsive forces for Right Rail (RR) (Quadrant 1,4)

%O1
row0RR = 0.15; %region of influence for Left Rail

%treating rail as line segment, finding shortest distance to right rail
P = [0.38;0.7;0.275];
Q = [0.38;-0.8;0.275];

%Finding nearest point on line segment
dist = dot((P-Q),(Q-O1C))/dot((P-Q),(P-Q));
NP = Q - dist*(P-Q);

%calculate distance of O1 to nearest point on right rail line segment
RowOfO1CRR = norm(O1C - NP);

%determine whether O1 falls in the region of influnce or not
if RowOfO1CRR <= row0RR
    GradOfO1CRR = (O1C - NP)/(norm(O1C-NP));
    FrepRightRailO1 = aeta2*((1/RowOfO1CRR) - (1/row0RR))*(1/RowOfO1CRR^2)*GradOfO1CRR;
else
    %if outside of region of influence set force to 0
    FrepRightRailO1 = [0;0;0]; 
end

%-------------------------------------------------------------------------%
%Repeat calculation for repulsive force for O2

%Finding nearest point on line segment
dist = dot((P-Q),(Q-O2C))/dot((P-Q),(P-Q));
NP = Q - dist*(P-Q);

%calculate distance of O2 to nearest point on right rail line segment
RowOfO2CRR = norm(O2C - NP);

%determine whether O2 falls in the region of influnce or not
if RowOfO2CRR <= row0RR
    GradOfO2CRR = (O2C - NP)/(norm(O2C-NP));
    FrepRightRailO2 = aeta2*((1/RowOfO2CRR) - (1/row0RR))*(1/(RowOfO2CRR^2))*GradOfO2CRR;
else
    %if outside of region of influence set force to 0
    FrepRightRailO2 = [0;0;0]; 
end

%-------------------------------------------------------------------------%
%Repeat calculation for repulsive force for O3

%Finding nearest point on line segment
dist = dot((P-Q),(Q-O3C))/dot((P-Q),(P-Q));
NP = Q - dist*(P-Q);

%calculate distance of O3 to nearest point on right rail line segment
RowOfO3CRR = norm(O3C - NP);

%determine whether O3 falls in the region of influnce or not
if RowOfO3CRR <= row0RR
    GradOfO3CRR = (O3C - NP)/(norm(O3C-NP));
    FrepRightRailO3 = aeta2*((1/RowOfO3CRR) - (1/row0RR))*(1/(RowOfO3CRR^2))*GradOfO3CRR;
else
    %if outside of region of influence set force to 0
    FrepRightRailO3 = [0;0;0]; 
end

%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%
%Determine Repulsive forces for Structural Pole

%O1
row0P = 0.15; %region of influence for pole (taken from center of pole)

%treating pole as line segment through center of pole
P = [0;0.65;1];
Q = [0;0.65;-0.8];

%Finding nearest point on line segment
dist = dot((P-Q),(Q-O1C))/dot((P-Q),(P-Q));
NP = Q - dist*(P-Q);

%calculate distance of O1 to nearest point on pole line segment
RowOfO1CP = norm(O1C - NP);

%determine whether O1 falls in the region of influence or not
if RowOfO1CP <= row0P
    GradOfO1CP = (O1C - NP)/(norm(O1C-NP));
    FrepPoleO1 = aeta3*((1/RowOfO1CP) - (1/row0P))*(1/RowOfO1CP^2)*GradOfO1CP;
else
    %if outside of region of influence set force to 0
    FrepPoleO1 = [0;0;0]; 
end

%-------------------------------------------------------------------------%
%Repeat calculation for repulsive force for O2

%Finding nearest point on line segment
dist = dot((P-Q),(Q-O2C))/dot((P-Q),(P-Q));
NP = Q - dist*(P-Q);

%calculate distance of O2 to nearest point on pole line segment
RowOfO2CP = norm(O2C - NP);

%determine whether O2 falls in the region of influnce or not
if RowOfO2CP <= row0P
  
    GradOfO2CP = (O2C - NP)/(norm(O2C-NP));
    FrepPoleO2 = aeta3*((1/RowOfO2CP) - (1/row0P))*(1/RowOfO2CP^2)*GradOfO2CP;
else
    %if outside of region of influence set force to 0
    FrepPoleO2 = [0;0;0]; 
end

%-------------------------------------------------------------------------%
%Repeat calculation for repulsive force for O3

%Finding nearest point on line segment
dist = dot((P-Q),(Q-O3C))/dot((P-Q),(P-Q));
NP = Q - dist*(P-Q);

%calculate distance of O3 to nearest point on pole line segment
RowOfO3CP = norm(O3C - NP);

%determine whether O3 falls in the region of influnce or not
if RowOfO3CP <= row0P
    GradOfO3CP = (O3C - NP)/(norm(O3C-NP));
    FrepPoleO3 = aeta3*((1/RowOfO3CP) - (1/row0P))*(1/(RowOfO3CP^2))*GradOfO3CP;
else
    %if outside of region of influence set force to 0
    FrepPoleO3 = [0;0;0]; 
end

%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%
%Sum the repulsive forces from each of the obstacles and return total
%repulsive forces on O1,O2,O3

FrepO1 = FrepLeftRailO1 + FrepRightRailO1 + FrepPoleO1;
FrepO2 = FrepLeftRailO2 + FrepRightRailO2 + FrepPoleO2;
FrepO3 = FrepLeftRailO3 + FrepRightRailO3 + FrepPoleO3;

end