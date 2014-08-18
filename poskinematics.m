clc;
clear all;

global l m an alphan Torques rn V1 V2 V3 V4 V5 V6 U1 U2 U3 U4 U5 U6 sn payload;

theta1 = 50*pi/180; %radians
theta2 = 30*pi/180;
r3 = 0.2;
theta4 = 5*pi/180;
theta5 = 15*pi/180;
theta6 = 30*pi/180;
l = 0.1;
m = 0.1;

an = zeros(6,1);
rn = [0; l; r3; 0; 0; m];
thetan = [theta1; theta2; 0; theta4; theta5; theta6];
%alphan = [90; 90; 0; 90; 90; 0];
alphan = [pi/2; pi/2; 0; pi/2; pi/2; 0];
sn = zeros(3,6);

R60 = zeros(3);
r60 = zeros(3,1);

for i=1:6
    %UN(:,:,i) = [cos(thetan(i)) -sin(thetan(i)) 0; sin(thetan(i)) cos(thetan(i)) 0; 0 0 1];
    %VN(:,:,i) = [1 0 0; 0 cos(alphan(i)) -sin(alphan(i)); 0 sin(alphan(i)) cos(alphan(i))];
    sn(:,1,i) = [an(i); 0; rn(i)];
end

%% Forward Kinematics

R60 = (U1)*(V1)*(U2)*(V2)*(U3)*(V3)*(U4)*(V4)*(U5)*(V5)*(U6)*(V6);
r60 = R60*sn(:,6) + (U1)*(V1)*(U2)*(V2)*(U3)*sn(:,3) + (U1)*(V1)*(U2)*sn(:,2);

%% Inverse Kinematics

%Required data variables
q = zeros(3,1);
AllThetas = zeros(6,8);
p = zeros(3);
q = r60 - R60*sn(:,1,6);
A = -q(2);
B = q(1);
C = -l;
Theta1A = 0;
Theta1B = 0;
r3Ap = 0; %r3A- = -r3Ap (so no need to add another variable)
r3Bp = 0;
Theta2Ap = 0;
Theta2An = 0;
Theta2Bp = 0;
Theta2Bn = 0;
Theta4A = 0;
Theta4B = 0;
Theta5A = 0;
Theta5B = 0;
Theta6A = 0;
Theta6B = 0;

a = C-A;
b = 2*B;
c = C+A;

% Alternative to making our own roots definition
% roots([C-A 2*B C+A])
% if (isscalar(ans)) %answer is a scalar - meaning that it is a linear
% eqn
%     ans(2) = ans(1)
% end
% Need to add a condition to remove small imaginary parts

if (abs(b^2 - 4*a*c) <= 1e-5)
    ans1 = -b/(2*a);
    ans2 = ans1;
elseif (a == 0)
        ans1 = -c/b;
        ans2 = ans1;
else
ans1 = (-b + sqrt(b^2 - 4*a*c))/(2*a);
ans2 = (-b - sqrt(b^2 - 4*a*c))/(2*a);
end

cosans = (1-ans1^2)/(1+ans1^2);
sinans = (2*ans1)/(1+ans1^2);

Theta1A = atan2(sinans, cosans);

cosans = (1-ans2^2)/(1+ans2^2);
sinans = (2*ans2)/(1+ans2^2);

Theta1B = atan2(sinans, cosans);

AllThetas(1,1:4) = Theta1A;
AllThetas(1,5:8) = Theta1B

temp1 = q(1)*cos(Theta1A) + q(2)*sin(Theta1A);
r3Ap = sqrt(temp1^2 + q(3)^2)
cosans = -q(3)/r3Ap; %Why keep r3Ap here? If it's non-zero it will cancel out, if it's zero it will create problems
sinans = temp1/r3Ap;
Theta2Ap = atan2(sinans, cosans); %Here even if r3An is used, both r3Ans will cancel and thus we get the same theta2A as r3Ap!
Theta2An = atan2(-sinans, -cosans);


temp1 = q(1)*cos(Theta1B) + q(2)*sin(Theta1B);
r3Bp = sqrt(temp1^2 + q(3)^2)
cosans = -q(3)/r3Bp; %Why keep r3Bp here? If it's non-zero it will cancel out, if it's zero it will create problems
sinans = temp1/r3Bp;
Theta2Bp = atan2(sinans, cosans); %Here even if r3Bn is used, both r3Bns will cancel and thus we get the same theta2B as r3Bp!
Theta2Bn = atan2(-sinans, -cosans);

AllThetas(2,:) = [Theta2Ap, Theta2Ap, Theta2An, Theta2An, Theta2Bp, Theta2Bp, Theta2Bn, Theta2Bn];
AllThetas(3,:) = [r3Ap, r3Ap, -r3Ap, -r3Ap, r3Bp, r3Bp, -r3Bp, -r3Bp]

% Thus, we now have all 4 solutions of theta1, theta2 and r3 in the matrix
% AllThetas

p = VN(:,:,3)'*UN(:,:,3)'*VN(:,:,2)'*UN(:,:,2)'*VN(:,:,1)'*UN(:,:,1)'*R60; %UNs, VNs not changed from fwd kinematics section

a = p(2,3);
b = 2*p(1,3);
c = -p(2,3);

if (abs(b^2 - 4*a*c) <= 1e-5)
    ans1 = -b/(2*a);
    ans2 = ans1;
elseif (a == 0)
        ans1 = -c/b;
        ans2 = ans1;
else
ans1 = (-b + sqrt(b^2 - 4*a*c))/(2*a);
ans2 = (-b - sqrt(b^2 - 4*a*c))/(2*a);
end

cosans = (1-ans1^2)/(1+ans1^2);
sinans = (2*ans1)/(1+ans1^2);

Theta4A = atan2(sinans, cosans);

cosans = (1-ans2^2)/(1+ans2^2);
sinans = (2*ans2)/(1+ans2^2);

Theta4B = atan2(sinans, cosans);

sinans = p(1,1)*sin(Theta4A) - p(2,1)*cos(Theta4A); %is it p(2,1) or p(1,2)??
cosans = p(1,2)*sin(Theta4A) - p(2,2)*cos(Theta4A);

Theta6A = atan2(sinans, cosans);

sinans = p(1,1)*sin(Theta4B) - p(2,1)*cos(Theta4B); %is it p(2,1) or p(1,2)??
cosans = p(1,2)*sin(Theta4B) - p(2,2)*cos(Theta4B);

Theta6B = atan2(sinans, cosans);

sinans = p(1,3)*cos(Theta4A) + p(2,3)*sin(Theta4A);
cosans = -p(3,3);

Theta5A = atan2(sinans, cosans);

sinans = p(1,3)*cos(Theta4B) + p(2,3)*sin(Theta4B);
cosans = -p(3,3);

Theta5B = atan2(sinans, cosans);

AllThetas(4,:) = [Theta4A, Theta4B, Theta4A, Theta4B, Theta4A, Theta4B, Theta4A, Theta4B];
AllThetas(5,:) = [Theta5A, Theta5B, Theta5A, Theta5B, Theta5A, Theta5B, Theta5A, Theta5B];
AllThetas(6,:) = [Theta6A, Theta6B, Theta6A, Theta6B, Theta6A, Theta6B, Theta6A, Theta6B]