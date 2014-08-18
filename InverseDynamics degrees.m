function [JointTorques] = InverseDynamics(theta, thetaDot, thetaDDot, gravity, msphere)

global l m an alphan Torques rn V1 V2 V3 V4 V5 V6 U1 U2 U3 U4 U5 U6 sn payload Rpayload;

%clc;
%clear all;

theta1 = 50;
theta2 = 30;
r3 = 0.2;
theta4 = 5;
theta5 = 15;
theta6 = 30;
%g = -9.82;
g = gravity;

% thetan = [theta1; theta2; 0; theta4; theta5; theta6];
% thetadotn = [1; 2; 1; 4; 3; 1; 1];
% thetaDdotn = [0.2; 0.3; 0.1; 0.01; 0.6; 1];

thetan = theta;
thetadotn = thetaDot;
thetaDdotn = thetaDDot;

rn = [0; l; r3; 0; 0; m];
alphan = [90; 90; 0; 90; 90; 0];
%sn = zeros(3,6); %3x1 vectors = [an; 0 ; rn]; NOT THE SAME AS si0, which are spatial vectors describing the 6 joints

R60 = zeros(3);
r60 = zeros(3,1);

%% Calculation of UNs and VNs

J = [1 0 0; 0 0 -1; 0 1 0]; %V for alpha = +90 degrees

for i=1:6
    sn(:,i) = [an(i); 0; rn(i)]; %Do NOT rename to s10, s20 etc. to avoid confusion. There is also a 6x1 vector s10, s20 etc. which is named as si0(:,i)
end

U1 = [cosd(thetan(1)) -sind(thetan(1)) 0; sind(thetan(1)) cosd(thetan(1)) 0; 0 0 1];
U2 = [cosd(thetan(2)) -sind(thetan(2)) 0; sind(thetan(2)) cosd(thetan(2)) 0; 0 0 1];
U3 = eye(3);
U4 = [cosd(thetan(4)) -sind(thetan(4)) 0; sind(thetan(4)) cosd(thetan(4)) 0; 0 0 1];
U5 = [cosd(thetan(5)) -sind(thetan(5)) 0; sind(thetan(5)) cosd(thetan(5)) 0; 0 0 1];
U6 = [cosd(thetan(6)) -sind(thetan(6)) 0; sind(thetan(6)) cosd(thetan(6)) 0; 0 0 1];

%% Forward Kinematics

R60 = (U1)*(V1)*(U2)*(V2)*(U3)*(V3)*(U4)*(V4)*(U5)*(V5)*(U6)*(V6);
r60 = R60*sn(:,6) + (U1)*(V1)*(U2)*(V2)*(U3)*sn(:,3) + (U1)*(V1)*(U2)*sn(:,2);

%% Calculation of Iiistar
% Iiistar = Moment of inertia, I, of link i in frame istar
% Only links 2,3 and 6 have a non-zero mass.
mass = 0.4948; %kg mass of a sample link. m2 = 2m, m3 = m6 = m.
Iiistar = zeros(6,6,6);

C22 = [0; -l/4; l/4];
C33 = [0; 0; -m/2];
C66 = [0; 0; (-m/2)*(mass/(mass+msphere))];

dist = -m/2 - C66(3);

Ixxlink6 = 5.391*10^-4 + mass*dist^2;
Iyylink6 = 5.391*10^-4 + mass*dist^2;
Izzlink6 = 2.53585*10^-4;

Ixxload = (2/5)*msphere*Rpayload*Rpayload + msphere*C66(3)*C66(3);
Iyyload = (2/5)*msphere*Rpayload*Rpayload + msphere*C66(3)*C66(3);
Izzload = (2/5)*msphere*Rpayload*Rpayload;

I66starSTAR = [Ixxload+Ixxlink6 0 0; 0 Iyyload+Iyylink6 0; 0 0 Izzload+Izzlink6];

Iiistar(1:3,4:6,2) = 2*mass*eye(3);
Iiistar(4:6,1:3,2) = 10^-4*[23.152 0 0; 0 14.112 -6.185; 0 -6.185 14.112];

Iiistar(1:3,4:6,3) = mass*eye(3);
Iiistar(4:6,1:3,3) = 10^-4*[5.391 0 0; 0 5.391 0; 0 0 2.536];

Iiistar(1:3,4:6,6) = (mass+msphere)*eye(3);
Iiistar(4:6,1:3,6) = I66starSTAR;

%% Calculation of si0s

si0 = zeros(6);
rhoi0 = zeros(3,6);
Wi0 = zeros(3,6);

Wi0(:,1) = [0; 0; 1];
Wi0(:,2) = (U1)*(V1)*[0; 0; 1];
Wi0(:,3) = (U1)*(V1)*(U2)*(V2)*[0; 0; 1];
Wi0(:,4) = (U1)*(V1)*(U2)*(V2)*(U3)*(V3)*[0; 0; 1];
Wi0(:,5) = (U1)*(V1)*(U2)*(V2)*(U3)*(V3)*(U4)*(V4)*[0; 0; 1];
Wi0(:,6) = (U1)*(V1)*(U2)*(V2)*(U3)*(V3)*(U4)*(V4)*(U5)*(V5)*[0; 0; 1];

rhoi0(:,2) = (U1)*sn(:,1) + rhoi0(:,1);%rhoi0(1) = 0
rhoi0(:,3) = (U1)*(V1)*(U2)*sn(:,2) + rhoi0(:,2);
rhoi0(:,4) = (U1)*(V1)*(U2)*(V2)*(U3)*sn(:,3) + rhoi0(:,3);
rhoi0(:,5) = (U1)*(V1)*(U2)*(V2)*(U3)*(V3)*(U4)*sn(:,4) + rhoi0(:,4);
rhoi0(:,6) = (U1)*(V1)*(U2)*(V2)*(U3)*(V3)*(U4)*(V4)*(U5)*sn(:,5) + rhoi0(:,5);

si0(:,1) = [Wi0(:,1); cross(rhoi0(:,1),Wi0(:,1))];
si0(:,2) = [Wi0(:,2); cross(rhoi0(:,2),Wi0(:,2))];
si0(:,3) = [0; 0; 0; Wi0(:,3)];
si0(:,4) = [Wi0(:,4); cross(rhoi0(:,4),Wi0(:,4))];
si0(:,5) = [Wi0(:,5); cross(rhoi0(:,5),Wi0(:,5))];
si0(:,6) = [Wi0(:,6); cross(rhoi0(:,6),Wi0(:,6))];

%% Calculation of Xistar0s and Ii0s
% Only need to calculate Xistar0 for non-zero moments of inertia matrices
% (i.e. links 2,3,6)

X2star0 = zeros(6);
X3star0 = zeros(6);
X6star0 = zeros(6);
Ri0 = zeros(3,3,6);

Ri0(:,:,1) = (U1)*(V1);
Ri0(:,:,2) = (U1)*(V1)*(U2)*(V2);
Ri0(:,:,3) = (U1)*(V1)*(U2)*(V2)*(U3)*(V3);
Ri0(:,:,4) = (U1)*(V1)*(U2)*(V2)*(U3)*(V3)*(U4)*(V4);
Ri0(:,:,5) = (U1)*(V1)*(U2)*(V2)*(U3)*(V3)*(U4)*(V4)*(U5)*(V5);
Ri0(:,:,6) = R60; %R60 is the same as (U1)*(V1)*(U2)*(V2)*(U3)*(V3)*(U4)*(V4)*(U5)*(V5)*(U6)*(V6)

% C22, C33, C66 defined before, for calculation of I66star

C20 = Ri0(:,:,2)*C22 + rhoi0(:,3);
C30 = Ri0(:,:,3)*C33 + rhoi0(:,4);
C60 = Ri0(:,:,6)*C66 + r60; %r60 is effectively Ri0(:,:,7), which is the position of end effector in frame 0

C20cross = [0 -C20(3) C20(2); C20(3) 0 -C20(1); -C20(2) C20(1) 0];
C30cross = [0 -C30(3) C30(2); C30(3) 0 -C30(1); -C30(2) C30(1) 0];
C60cross = [0 -C60(3) C60(2); C60(3) 0 -C60(1); -C60(2) C60(1) 0];

X2star0(1:3,1:3) = Ri0(:,:,2);
X2star0(4:6,4:6) = Ri0(:,:,2);
X2star0(4:6,1:3) = C20cross*Ri0(:,:,2);

X3star0(1:3,1:3) = Ri0(:,:,3);
X3star0(4:6,4:6) = Ri0(:,:,3);
X3star0(4:6,1:3) = C30cross*Ri0(:,:,3);

X6star0(1:3,1:3) = Ri0(:,:,6);
X6star0(4:6,4:6) = Ri0(:,:,6);
X6star0(4:6,1:3) = C60cross*Ri0(:,:,6);

X2star0T = zeros(6);
X3star0T = zeros(6);
X6star0T = zeros(6);

X2star0T = SpatialTranspose(X2star0);
% X2star0T(1:3,1:3) = X2star0(4:6,4:6)';
% X2star0T(1:3,4:6) = X2star0(1:3,4:6)';
% X2star0T(4:6,1:3) = X2star0(4:6,1:3)';
% X2star0T(4:6,4:6) = X2star0(1:3,1:3)';

X3star0T = SpatialTranspose(X3star0);
% X3star0T(1:3,1:3) = X3star0(4:6,4:6)';
% X3star0T(1:3,4:6) = X3star0(1:3,4:6)';
% X3star0T(4:6,1:3) = X3star0(4:6,1:3)';
% X3star0T(4:6,4:6) = X3star0(1:3,1:3)';

X6star0T = SpatialTranspose(X6star0);
% X6star0T(1:3,1:3) = X6star0(4:6,4:6)';
% X6star0T(1:3,4:6) = X6star0(1:3,4:6)';
% X6star0T(4:6,1:3) = X6star0(4:6,1:3)';
% X6star0T(4:6,4:6) = X6star0(1:3,1:3)';

I20 = X2star0*Iiistar(:,:,2)*X2star0T;
I30 = X3star0*Iiistar(:,:,3)*X3star0T;
I60 = X6star0*Iiistar(:,:,6)*X6star0T;

%% Velocity and Acceleration recursions (calculation of vi0s and ai0s)

v00 = zeros(6,1);
v10 = v00 + si0(:,1)*thetadotn(1);
v20 = v10 + si0(:,2)*thetadotn(2);
v30 = v20 + si0(:,3)*thetadotn(3);
v40 = v30 + si0(:,4)*thetadotn(4);
v50 = v40 + si0(:,5)*thetadotn(5);
v60 = v50 + si0(:,6)*thetadotn(6);

v10Cross = SpatialCross(v10);
v20Cross = SpatialCross(v20);
v30Cross = SpatialCross(v30);
v40Cross = SpatialCross(v40);
v50Cross = SpatialCross(v50);
v60Cross = SpatialCross(v60);

a00 = [0 0 0 0 0 g]';
a10 = a00 + v10Cross*si0(:,1)*thetadotn(1) + si0(:,1)*thetaDdotn(1);
a20 = a10 + v20Cross*si0(:,2)*thetadotn(2) + si0(:,2)*thetaDdotn(2);
a30 = a20 + v30Cross*si0(:,3)*thetadotn(3) + si0(:,3)*thetaDdotn(3);
a40 = a30 + v40Cross*si0(:,4)*thetadotn(4) + si0(:,4)*thetaDdotn(4);
a50 = a40 + v50Cross*si0(:,5)*thetadotn(5) + si0(:,5)*thetaDdotn(5);
a60 = a50 + v60Cross*si0(:,6)*thetadotn(6) + si0(:,6)*thetaDdotn(6);

%% Force recursions (calculation of fi0 and fstari0)

endEffectorForce = zeros(6,1);

fstar60 = I60*a60 + v60Cross*I60*v60;
fstar50 = zeros(6,1);
fstar40 = zeros(6,1);
fstar30 = I30*a30 + v30Cross*I30*v30;
fstar20 = I20*a20 + v20Cross*I20*v20;
fstar10 = zeros(6,1);

f70 = endEffectorForce; %zero force on end effector, for now
f60 = f70 + fstar60;
f50 = f60 + fstar50;
f40 = f50 + fstar40;
f30 = f40 + fstar30;
f20 = f30 + fstar20;
f10 = f20 + fstar10;

%% Torque calculations

TauFF1 = SpatialDot(si0(:,1),f10);
TauFF2 = SpatialDot(si0(:,2),f20);
TauFF3 = SpatialDot(si0(:,3),f30);
TauFF4 = SpatialDot(si0(:,4),f40);
TauFF5 = SpatialDot(si0(:,5),f50);
TauFF6 = SpatialDot(si0(:,6),f60);

JointTorques = [TauFF1 TauFF2 TauFF3 TauFF4 TauFF5 TauFF6]';
end