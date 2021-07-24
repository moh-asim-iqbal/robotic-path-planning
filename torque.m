function torqueVec = torque(CurrQ,EndQ,zeta1,zeta2,zeta3,d,aeta1,aeta2,aeta3)
%torque function that calls attractive func and repulsive func to determine
%the attractive and repulsive forces for O1,O2,O3. Then calls the jacobian
%function and converts the forces into torques in the configuration space.

%Determine attractive forces for current configuration
[FattO1, FattO2, FattO3] = attractive(CurrQ,EndQ,zeta1,zeta2,zeta3,d);

%Determine repuslive forces for current configuration
[FrepO1, FrepO2, FrepO3] = repulsive(CurrQ,aeta1,aeta2,aeta3);

%Determine jacobian matricies for O1,O2,O3
[JVO1, JVO2, JVO3] = jacobianscriptMOD(CurrQ(1),CurrQ(2),CurrQ(3));

%Determine attractive torques
TattO1 = transpose(JVO1)*FattO1;
TattO2 = transpose(JVO2)*FattO2;
TattO3 = transpose(JVO3)*FattO3;

%Determine repulsive torques
TrepO1 = transpose(JVO1)*FrepO1;
TrepO2 = transpose(JVO2)*FrepO2;
TrepO3 = transpose(JVO3)*FrepO3;

%Determine total torques for O1,O2,O3
TtotalO1 = TattO1 + TrepO1;
TtotalO2 = TattO2 + TrepO2;
TtotalO3 = TattO3 + TrepO3;

%return result
torqueVec = TtotalO1 + TtotalO2 + TtotalO3;


end