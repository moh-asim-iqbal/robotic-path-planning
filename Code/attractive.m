function [FattO1, FattO2, FattO3] = attractive(CurrQ,EndQ,zeta1,zeta2,zeta3,d)
%function that takes current configuratin, goal configuration and
%calculates attractive forces on O1,O2,O3. Can be tweaked by zeta value.
%Parabolic or Conic well used depends on d

%Transformation matrix for final configuration
[T01, T02, T03] = tmatrixscriptMOD(EndQ(1),EndQ(2),EndQ(3));

%Get OF (O for final configuration
O1F = T01(1:3,4);
O2F = T02(1:3,4);
O3F = T03(1:3,4);

OFMat = [O1F,O2F,O3F];

%calculate attractive for for each joint

%Transformation matricies for each joint at curr position
[T01C, T02C, T03C] = tmatrixscriptMOD(CurrQ(1),CurrQ(2),CurrQ(3));

%Frame origins for each joint at current config
O1C = T01C(1:3,4);
O2C = T02C(1:3,4);
O3C = T03C(1:3,4);

OCMat = [O1C,O2C,O3C];

FattMatrix = [];

%calculate attractive forces for each frame using appropriate attractive
%field
for i = 1:3
    %calculate norm between current frame and goal frame
    goalDist = norm(OCMat(:,i)-OFMat(:,i));
    
    %set zeta value depending on which frame being assessed
    if i == 1
        zeta = zeta1;
    elseif i == 2
        zeta = zeta2;
    else
        zeta = zeta3;
    end
    
    %calculate attractive force depending on goaldistance and d
    if goalDist <= d
        %Use parabolic potential
        f = -zeta*(OCMat(:,i)-OFMat(:,i));
        FattMatrix = [FattMatrix,f];
    else
        %Use conic potential
        f = -d*zeta*((OCMat(:,i)-OFMat(:,i))/goalDist);
        FattMatrix = [FattMatrix,f];
    end
  
end
%Seperate FattMatrix into attractive forces for each frame
FattO1 = FattMatrix(:,1);
FattO2 = FattMatrix(:,2);
FattO3 = FattMatrix(:,3);


end