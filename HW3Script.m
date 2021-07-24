format short
close all
clear all
%Determine initial configuration 
Q1 = [10,0,0.4];


%Goal Point in Workspace
xG = -0.59;
yG = 0.06;
zG = 0.3;

%Get configuration for this point using inverse kinematics
[sol1, sol2] = invkinscript(xG,yG,zG);

%taking solution 1 in this case as elbow down configuration desired
QG = sol1;

disp('Starting Configuration: ');
disp(Q1);
disp('Final Configuration: ');
disp(QG);

%specify various parameters for torque calculation
qCurr = Q1; %Passing starting configuration to algorithm
EndQ = QG;  %Passing ending configuration to algorithm
zeta1 = 2; %attractive force constant for O1, O1 set as leader
zeta2 = 0.75; %attractive force constant for O2
zeta3 = 0.75; %attractive force constant for O3
d = 0.01; %distance in cm that changes conic potential to parabolic
aeta1 = 0.25; %repulsive constant for Left Conveyor Rail (Quadrant 2,3)
aeta2 = 0.5; %repulsive constant for Right Conveyor Rail (Quadrant 1,4)
aeta3 = 0.5 ;%repulsive constant for Structural Pole (Quadrant 1,2)

%specify various parameters for gradient decent algorithm
alpha1 = 3; %step value for theta1
alpha2 = 3; %step value for theta2
alpha3 = 0.1; %step value for d3
epsilon = 3; 
epsilonM = 0.1;
configMat = Q1; %matrix to hold configuration list
FattMat =[];

%Gradient Descent Algorithm
for i =1:1000
    %calculate the unit torque vector
    torqVec = torque(qCurr,EndQ,zeta1,zeta2,zeta3,d,aeta1,aeta2,aeta3);
    unitTorqVec = torqVec/(norm(torqVec));
   
    %Save previous configuration to make sure we aren't in local minimum
    qPrev = qCurr;
    
    %See if current configuration is close enough to goal configuration
    if norm(qCurr - QG) > epsilon
        
        %Determine step for each joint
        stepVec =  [alpha1*unitTorqVec(1),alpha2*unitTorqVec(2),alpha3*unitTorqVec(3)];
        
        %Ensuring Step is within bounds of rotary joint theta1
        if qCurr(1) + stepVec(1) > 360
            qCurr = [360, qCurr(2), qCurr(3)];
        elseif qCurr(1) + stepVec(1) < 0
            qCurr = [0, qCurr(2), qCurr(3)];
        else
            qCurr = [qCurr(1)+stepVec(1), qCurr(2), qCurr(3)];
        end
        
        %Ensuring Step is within bounds of rotary joint theta2
        if qCurr(2) + stepVec(2) > 180
            qCurr = [qCurr(1), 180, qCurr(3)];
        elseif qCurr(2) + stepVec(2) < -180
            qCurr = [qCurr(1), -180, qCurr(3)];
        else
            qCurr = [qCurr(1),qCurr(2)+stepVec(2),qCurr(3)];
        end
        
        %Ensuring Step is within bounds of prismatic joint d3
        if qCurr(3) + stepVec(3) > 0.4
            qCurr = [qCurr(1),qCurr(2),0.4];
        elseif qCurr(3) + stepVec(3) < 0
            qCurr = [qCurr(1),qCurr(2),0];
        else
            qCurr = [qCurr(1),qCurr(2),qCurr(3)+stepVec(3)];
        end
        
    %Current config is close enough to goal, so exit algorithm
    else
        configMat = [configMat;qCurr];
        break
    end
    
   %Add current configuration to list and keep looping towards goal
    configMat = [configMat;qCurr];
    
    %Determine if stuck in local minimum
    if i >1 && (norm(qCurr - qPrev) < epsilonM)
        disp('Manipulator in Local Minima');
        
        %Take a random walk
        walkConstant = 2*(rand()-0.5);
        randomStepVec = transpose(walkConstant*unitTorqVec);
       
        %Verify random step is within bounds of theta1
        if (qCurr(1) + randomStepVec(1)) > 360 ||(qCurr(1) + randomStepVec(1)) < 0
            qCurr = [qCurr(1) - randomStepVec(1), qCurr(2),qCurr(3)];
        elseif (qCurr(1) - randomStepVec(1)) > 360 ||(qCurr(1) - randomStepVec(1)) < 0
            qCurr = [qCurr(1) + randomStepVec(1), qCurr(2),qCurr(3)];
        else
            qCurr = [qCurr(1) - randomStepVec(1), qCurr(2),qCurr(3)];
        end
        
        %Verify random step is not going past joint limits of theta2
        if (qCurr(2) + randomStepVec(2)) > 180 ||(qCurr(2) + randomStepVec(2)) < -180
            qCurr = [qCurr(1), qCurr(2)- randomStepVec(2),qCurr(3)];
        elseif (qCurr(2) - randomStepVec(2)) > 180 ||(qCurr(2) - randomStepVec(2)) < -180
            qCurr = [qCurr(1), qCurr(2)+ randomStepVec(2),qCurr(3)];
        else
            qCurr = [qCurr(1), qCurr(2) - randomStepVec(2),qCurr(3)];
        end
        
        %Using modulo operator to ensure random step is within bounds of d3
        qCurr = [qCurr(1), qCurr(2),mod(qCurr(3)- randomStepVec(3),0.4)];

    end
    
end
