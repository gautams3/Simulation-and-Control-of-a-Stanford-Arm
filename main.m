% Main simulation file
% Gautam Salhotra - Spring 2012

clear all;
clc;

global l m an alphan V1 V2 V3 V4 V5 V6 U3 sn payload Rpayload ControllerTimestep;
global MMatrix;

%ThetaInitial = [0.2014; pi/2; 0.4899; 0; -pi/2; -2.9402]; %1st branch of inv kinematics 
ThetaInitial = [2.9042; -pi/2; 0.4899; 0; pi/2; -0.2014]; %5th branch
%ThetaFinal = [-0.49; -2.0344; 0.4472; 0; 1.1071; 2.6516]; %1st branch
% ThetaFinal = [-0.49; -2.0344; 0.4472; 0; 1.1071; 2.6516]; %5th branch
ThetaFinal = [2.5648; -1.6172; 0.4856; 0; 1.5244; 0.0839]; %new thetaF for part 2

PolynomialCoeffsOfDesiredTheta = zeros(6);

% ErrorIntegral = zeros(6,1); %Initial error integral for all 6 position variables is zero
ControllerTimestep = 0.01; %100Hz
SimulationTimestep = 0.001; %1000Hz

J = [1 0 0; 0 0 -1; 0 1 0]; %V for alpha=+90 degrees
alphan = [pi/2; pi/2; 0; pi/2; pi/2; 0];
sn = zeros(3,6);
an = zeros(6,1);
payload = 0.4; %mass of sphere
Rpayload = 0.025; %0.05 dia sphere
l = 0.1;
m = 0.1;
% r3 = 0.2; %temporary
% rn = [0; l; r3; 0; 0; m];
% 
% 
% for i=1:6
%     sn(:,i) = [an(i); 0; rn(i)];
% end

V1 = J;
V2 = J;
V3 = eye(3);
V4 = J;
V5 = J;
V6 = eye(3);

U3 = eye(3);

%% Calculation of ThetaDesired, ThetaDotDesired, ThetaDotDesired

Theta1Desired = zeros(1001,1);
Theta1DotDesired = zeros(1001,1);
Theta1DoubleDotDesired = zeros(1001,1);

T = 10; %Final time
K = [1 0 0 0 0 0; 1 T T^2 T^3 T^4 T^5; 0 1 0 0 0 0; 0 1 2*T 3*T^2 4*T^3 5*T^4; 0 0 2 0 0 0; 0 0 2 6*T 12*T^2 20*T^3];
for (i=1:6)
PolynomialCoeffsOfDesiredTheta(:,i) = inv(K)*[ThetaInitial(i); ThetaFinal(i); 0; 0; 0; 0];
end

t = 0; %time variable
a = PolynomialCoeffsOfDesiredTheta(1,1);
b = PolynomialCoeffsOfDesiredTheta(2,1);
c = PolynomialCoeffsOfDesiredTheta(3,1);
d = PolynomialCoeffsOfDesiredTheta(4,1);
e = PolynomialCoeffsOfDesiredTheta(5,1);
f = PolynomialCoeffsOfDesiredTheta(6,1);

for i=1:1001
    t = (i-1)*ControllerTimestep;    
    Theta1Desired(i) = a + b*t + c*t^2 + d*t^3 + e*t^4 + f*t^5;
    Theta1DotDesired(i) = b + 2*c*t + 3*d*t^2 + 4*e*t^3 + 5*f*t^4;
    Theta1DoubleDotDesired(i) = 2*c + 6*d*t + 12*e*t^2 + 20*f*t^3;
end

Theta2Desired = zeros(1001,1);
Theta2DotDesired = zeros(1001,1);
Theta2DoubleDotDesired = zeros(1001,1);

t = 0; %time variable
a = PolynomialCoeffsOfDesiredTheta(1,2);
b = PolynomialCoeffsOfDesiredTheta(2,2);
c = PolynomialCoeffsOfDesiredTheta(3,2);
d = PolynomialCoeffsOfDesiredTheta(4,2);
e = PolynomialCoeffsOfDesiredTheta(5,2);
f = PolynomialCoeffsOfDesiredTheta(6,2);

for i=1:1001
    t = (i-1)*ControllerTimestep;
    Theta2Desired(i) = a + b*t + c*t^2 + d*t^3 + e*t^4 + f*t^5;
    Theta2DotDesired(i) = b + 2*c*t + 3*d*t^2 + 4*e*t^3 + 5*f*t^4;
    Theta2DoubleDotDesired(i) = 2*c + 6*d*t + 12*e*t^2 + 20*f*t^3;
end

Theta3Desired = zeros(1001,1);
Theta3DotDesired = zeros(1001,1);
Theta3DoubleDotDesired = zeros(1001,1); %This is actually r3, not theta3

t = 0; %time variable
a = PolynomialCoeffsOfDesiredTheta(1,3);
b = PolynomialCoeffsOfDesiredTheta(2,3);
c = PolynomialCoeffsOfDesiredTheta(3,3);
d = PolynomialCoeffsOfDesiredTheta(4,3);
e = PolynomialCoeffsOfDesiredTheta(5,3);
f = PolynomialCoeffsOfDesiredTheta(6,3);

for i=1:1001
    t = (i-1)*ControllerTimestep;
    Theta3Desired(i) = a + b*t + c*t^2 + d*t^3 + e*t^4 + f*t^5;
    Theta3DotDesired(i) = b + 2*c*t + 3*d*t^2 + 4*e*t^3 + 5*f*t^4;
    Theta3DoubleDotDesired(i) = 2*c + 6*d*t + 12*e*t^2 + 20*f*t^3;
end

Theta4Desired = zeros(1001,1);
Theta4DotDesired = zeros(1001,1);
Theta4DoubleDotDesired = zeros(1001,1);

t = 0; %time variable
a = PolynomialCoeffsOfDesiredTheta(1,4);
b = PolynomialCoeffsOfDesiredTheta(2,4);
c = PolynomialCoeffsOfDesiredTheta(3,4);
d = PolynomialCoeffsOfDesiredTheta(4,4);
e = PolynomialCoeffsOfDesiredTheta(5,4);
f = PolynomialCoeffsOfDesiredTheta(6,4);

for i=1:1001
    t = (i-1)*ControllerTimestep;
    Theta4Desired(i) = a + b*t + c*t^2 + d*t^3 + e*t^4 + f*t^5;
    Theta4DotDesired(i) = b + 2*c*t + 3*d*t^2 + 4*e*t^3 + 5*f*t^4;
    Theta4DoubleDotDesired(i) = 2*c + 6*d*t + 12*e*t^2 + 20*f*t^3;
end

Theta5Desired = zeros(1001,1);
Theta5DotDesired = zeros(1001,1);
Theta5DoubleDotDesired = zeros(1001,1);

t = 0; %time variable
a = PolynomialCoeffsOfDesiredTheta(1,5);
b = PolynomialCoeffsOfDesiredTheta(2,5);
c = PolynomialCoeffsOfDesiredTheta(3,5);
d = PolynomialCoeffsOfDesiredTheta(4,5);
e = PolynomialCoeffsOfDesiredTheta(5,5);
f = PolynomialCoeffsOfDesiredTheta(6,5);

for i=1:1001
    t = (i-1)*ControllerTimestep;
    Theta5Desired(i) = a + b*t + c*t^2 + d*t^3 + e*t^4 + f*t^5;
    Theta5DotDesired(i) = b + 2*c*t + 3*d*t^2 + 4*e*t^3 + 5*f*t^4;
    Theta5DoubleDotDesired(i) = 2*c + 6*d*t + 12*e*t^2 + 20*f*t^3;
end

Theta6Desired = zeros(1001,1);
Theta6DotDesired = zeros(1001,1);
Theta6DoubleDotDesired = zeros(1001,1);

t = 0; %time variable
a = PolynomialCoeffsOfDesiredTheta(1,6);
b = PolynomialCoeffsOfDesiredTheta(2,6);
c = PolynomialCoeffsOfDesiredTheta(3,6);
d = PolynomialCoeffsOfDesiredTheta(4,6);
e = PolynomialCoeffsOfDesiredTheta(5,6);
f = PolynomialCoeffsOfDesiredTheta(6,6);
for i=1:1001
    t = (i-1)*ControllerTimestep;
    Theta6Desired(i) = a + b*t + c*t^2 + d*t^3 + e*t^4 + f*t^5;
    Theta6DotDesired(i) = b + 2*c*t + 3*d*t^2 + 4*e*t^3 + 5*f*t^4;
    Theta6DoubleDotDesired(i) = 2*c + 6*d*t + 12*e*t^2 + 20*f*t^3;
end

%% Actual Simulation
ActualPos = zeros(1001,6); %only updates values every 0.01sec, not every 0.001sec
ActualVel = zeros(1001,6);
ActualAccn = zeros(1001,6);
ThetaDesired = [Theta1Desired Theta2Desired Theta3Desired Theta4Desired Theta5Desired Theta6Desired];
ThetaDotDesired = [Theta1DotDesired Theta2DotDesired Theta3DotDesired Theta4DotDesired Theta5DotDesired Theta6DotDesired];
ThetaDoubleDotDesired = [Theta1DoubleDotDesired Theta2DoubleDotDesired Theta3DoubleDotDesired Theta4DoubleDotDesired Theta5DoubleDotDesired Theta6DoubleDotDesired];
TauFF = zeros(6,1001);
TauFB = zeros(6,1001);
Error = zeros(6,1001);
ErrorDot = zeros(6,1001);
% AccnArray = zeros(6,1001);

clear Theta1Desired Theta2Desired Theta3Desired Theta4Desired Theta5Desired Theta6Desired;
clear Theta1DotDesired Theta2DotDesired Theta3DotDesired Theta4DotDesired Theta5DotDesired Theta6DotDesired;
clear Theta1DoubleDotDesired Theta2DoubleDotDesired Theta3DoubleDotDesired Theta4DoubleDotDesired Theta5DoubleDotDesired Theta6DoubleDotDesired;
clear a b c d e f t;

for(i=1:1001)
TauFF(:,i) = (InverseDynamics(ThetaDesired(i,:)',ThetaDotDesired(i,:)',ThetaDoubleDotDesired(i,:)',-9.82,0));
% AccnArray(:,i) = FwdDynamics(ThetaDesired(i,:)',ThetaDotDesired(i,:)', TauFF(:,i), 0);
end
% X1 = 1:1001;
% plot(X1,ThetaDoubleDotDesired(:,4)','rx', X1, AccnArray(4,:))

time = 0; % Simulation time variable
SimulationIndex = 0; %this should go from 0 to 10000, as SimulationTimestep is 0.001sec
ControllerIndex = 0; %this should go from 1 to 1001, as ControllerTimestep is 0.01sec
ActualPos(1,:) = ThetaInitial'; %ActualVel and ActualAccn initial are zero

Theta = ActualPos(1,:)'; %setting initial conditions for pos, vel, accn as column vectors
ThetaDot = ActualVel(1,:)';
ThetaDDot = ActualAccn(1,:)';
TotalTorque = 0; %Initialize total torque

for(SimulationIndex = 0:1:10000)
    
    if(rem(SimulationIndex,10)==0) %if time is a multiple of 0.01sec i.e. the controller timestep
        ControllerIndex = ControllerIndex + 1; %increase controller index
        ActualPos(ControllerIndex,:) = Theta'; %update row vector of pos, vel, accn for that time instant
        ActualVel(ControllerIndex,:) = ThetaDot';
        ActualAccn(ControllerIndex,:) = ThetaDDot';
        Error(:,ControllerIndex) = (ThetaDesired(ControllerIndex,:) - ActualPos(ControllerIndex,:))';
        ErrorDot(:,ControllerIndex) = (ThetaDotDesired(ControllerIndex,:) - ActualVel(ControllerIndex,:))';
        PointlessCalculation = FwdDynamics(Theta, ThetaDot, TotalTorque, payload); %so that MMatrix is updated
        TauFB(:,ControllerIndex) = MMatrix*ControlOutput(Error(:,ControllerIndex), ErrorDot(:,ControllerIndex)); %update new feedback torque by controller
        TotalTorque = TauFF(:,ControllerIndex) + TauFB(:,ControllerIndex);       
%         [TauFF(:,ControllerIndex) Error(:,ControllerIndex) ErrorDot(:,ControllerIndex) TauFB(:,ControllerIndex) TotalTorque]
    end
    
    ThetaDDot = FwdDynamics(Theta, ThetaDot, TotalTorque, payload); %Use current theta, thetadot, tau to calc thetaDDot
    [Theta, ThetaDot] = RungeKuttaFixedTime(Theta, ThetaDot, TotalTorque, payload, SimulationTimestep); %Calculate ThetaDot for next timestep
    time = time + SimulationTimestep %update time for next timestep
end