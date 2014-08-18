function [ThetaOut] = InverseKinematics(EndEffectorPosition, EndEffectorRotationMatrix)

%global l m an alphan rn V1 V2 V3 V4 V5 V6 U1 U2 U3 U4 U5 U6 sn;

R60 = EndEffectorRotationMatrix;
r60 = EndEffectorPosition;
%% Variables needed for offline calculation of Inverse Kinematics of initial and final position
l = 0.1;
m = 0.1;
r3 = 0.2; %temporary
an = zeros(6,1);
rn = [0; l; r3; 0; 0; m];
sn = zeros(3,6);
sn(:,6) = [an(6); 0; rn(6)];


J = [1 0 0; 0 0 -1; 0 1 0]; %V for alpha=+90 degrees
alphan = [pi/2; pi/2; 0; pi/2; pi/2; 0];
V1 = J;
V2 = J;
V3 = eye(3);
V4 = J;
V5 = J;
V6 = eye(3);

U3 = eye(3);

%%
%Required data variables
q = zeros(3,1); %Wrist position
AllThetas = zeros(6,8); %All 8 possible values for theta
p = zeros(3);
q = r60 - R60*sn(:,6);
A = -q(2);
B = q(1);
C = -l;
Theta1A = 0;
Theta1B = 0;
r3Ap = 0; %r3An = -r3Ap, so no need to add another variable
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

a = C-A; %Coefficients for A*cos(theta) + B*sin(theta) + C = 0 (Eqn 33)
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
AllThetas(1,5:8) = Theta1B;

temp1 = q(1)*cos(Theta1A) + q(2)*sin(Theta1A);
r3Ap = sqrt(temp1^2 + q(3)^2);
cosans = -q(3)/r3Ap;
sinans = temp1/r3Ap;
Theta2Ap = atan2(sinans, cosans);
Theta2An = atan2(-sinans, -cosans);

temp1 = q(1)*cos(Theta1B) + q(2)*sin(Theta1B);
r3Bp = sqrt(temp1^2 + q(3)^2);
cosans = -q(3)/r3Bp;
sinans = temp1/r3Bp;
Theta2Bp = atan2(sinans, cosans);
Theta2Bn = atan2(-sinans, -cosans);

AllThetas(2,:) = [Theta2Ap, Theta2Ap, Theta2An, Theta2An, Theta2Bp, Theta2Bp, Theta2Bn, Theta2Bn];
AllThetas(3,:) = [r3Ap, r3Ap, -r3Ap, -r3Ap, r3Bp, r3Bp, -r3Bp, -r3Bp];

% Thus, we now have all 4 solutions of theta1, theta2 and r3 in the matrix

U1 = [cos(Theta1B) -sin(Theta1B) 0; sin(Theta1B) cos(Theta1B) 0; 0 0 1]; %Taking the 5th soln
U2 = [cos(Theta2Bp) -sin(Theta2Bp) 0; sin(Theta2Bp) cos(Theta2Bp) 0; 0 0 1];
r3 = r3Bp;
sn(3,3) = r3;

p = (V3)'*(U3)'*(V2)'*(U2)'*(V1)'*(U1)'*R60;

A = -p(2,3); %Coefficients for A*cos(theta) + B*sin(theta) + C = 0 (Eqn 33)
B = p(1,3);
C = 0;

a = C-A;
b = 2*B;
c = C+A;

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
AllThetas(6,:) = [Theta6A, Theta6B, Theta6A, Theta6B, Theta6A, Theta6B, Theta6A, Theta6B];

U4 = [cos(Theta4A) -sin(Theta4A) 0; sin(Theta4A) cos(Theta4A) 0; 0 0 1];
U5 = [cos(Theta5A) -sin(Theta5A) 0; sin(Theta5A) cos(Theta5A) 0; 0 0 1];
U6 = [cos(Theta6A) -sin(Theta6A) 0; sin(Theta6A) cos(Theta6A) 0; 0 0 1];

ThetaOut = AllThetas;
end