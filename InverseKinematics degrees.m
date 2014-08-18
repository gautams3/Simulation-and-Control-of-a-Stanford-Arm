function [ThetaOut] = InverseKinematics(EndEffectorPosition, EndEffectorRotationMatrix)

global l m an alphan Torques rn V1 V2 V3 V4 V5 V6 U1 U2 U3 U4 U5 U6 sn payload;

R60 = EndEffectorRotationMatrix;
r60 = EndEffectorPosition;

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

Theta1A = atan2(sinans, cosans)*180/pi;

cosans = (1-ans2^2)/(1+ans2^2);
sinans = (2*ans2)/(1+ans2^2);

Theta1B = atan2(sinans, cosans)*180/pi;

AllThetas(1,1:4) = Theta1A;
AllThetas(1,5:8) = Theta1B;

temp1 = q(1)*cosd(Theta1A) + q(2)*sind(Theta1A);
r3Ap = sqrt(temp1^2 + q(3)^2);
cosans = -q(3)/r3Ap;
sinans = temp1/r3Ap;
Theta2Ap = atan2(sinans, cosans)*180/pi;
Theta2An = atan2(-sinans, -cosans)*180/pi;

temp1 = q(1)*cosd(Theta1B) + q(2)*sind(Theta1B);
r3Bp = sqrt(temp1^2 + q(3)^2);
cosans = -q(3)/r3Bp;
sinans = temp1/r3Bp;
Theta2Bp = atan2(sinans, cosans)*180/pi;
Theta2Bn = atan2(-sinans, -cosans)*180/pi;

AllThetas(2,:) = [Theta2Ap, Theta2Ap, Theta2An, Theta2An, Theta2Bp, Theta2Bp, Theta2Bn, Theta2Bn];
AllThetas(3,:) = [r3Ap, r3Ap, -r3Ap, -r3Ap, r3Bp, r3Bp, -r3Bp, -r3Bp];

% Thus, we now have all 4 solutions of theta1, theta2 and r3 in the matrix
% AllThetas

U1 = [cosd(Theta1A) -sind(Theta1A) 0; sind(Theta1A) cosd(Theta1A) 0; 0 0 1]; %Taking the first soln
U2 = [cosd(Theta2Ap) -sind(Theta2Ap) 0; sind(Theta2Ap) cosd(Theta2Ap) 0; 0 0 1];
r3 = r3Ap;
sn(3,3) = r3;

p = (V3)'*(U3)'*(V2)'*(U2)'*(V1)'*(U1)'*R60;

A = -p(2,3); %Coefficients for A*cos(theta) + B*sin(theta) + C = 0 (Eqn 33)
B = p(1,3);
C = 0;

a = C-A
b = 2*B
c = C+A

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

Theta4A = atan2(sinans, cosans)*180/pi;

cosans = (1-ans2^2)/(1+ans2^2);
sinans = (2*ans2)/(1+ans2^2);

Theta4B = atan2(sinans, cosans)*180/pi;

sinans = p(1,1)*sind(Theta4A) - p(2,1)*cosd(Theta4A); %is it p(2,1) or p(1,2)??
cosans = p(1,2)*sind(Theta4A) - p(2,2)*cosd(Theta4A);

Theta6A = atan2(sinans, cosans)*180/pi;

sinans = p(1,1)*sind(Theta4B) - p(2,1)*cosd(Theta4B); %is it p(2,1) or p(1,2)??
cosans = p(1,2)*sind(Theta4B) - p(2,2)*cosd(Theta4B);

Theta6B = atan2(sinans, cosans)*180/pi;

sinans = p(1,3)*cosd(Theta4A) + p(2,3)*sind(Theta4A);
cosans = -p(3,3);

Theta5A = atan2(sinans, cosans)*180/pi;

sinans = p(1,3)*cosd(Theta4B) + p(2,3)*sind(Theta4B);
cosans = -p(3,3);

Theta5B = atan2(sinans, cosans)*180/pi;

AllThetas(4,:) = [Theta4A, Theta4B, Theta4A, Theta4B, Theta4A, Theta4B, Theta4A, Theta4B];
AllThetas(5,:) = [Theta5A, Theta5B, Theta5A, Theta5B, Theta5A, Theta5B, Theta5A, Theta5B];
AllThetas(6,:) = [Theta6A, Theta6B, Theta6A, Theta6B, Theta6A, Theta6B, Theta6A, Theta6B];

U4 = [cosd(Theta4A) -sind(Theta4A) 0; sind(Theta4A) cosd(Theta4A) 0; 0 0 1];
U5 = [cosd(Theta5A) -sind(Theta5A) 0; sind(Theta5A) cosd(Theta5A) 0; 0 0 1];
U6 = [cosd(Theta6A) -sind(Theta6A) 0; sind(Theta6A) cosd(Theta6A) 0; 0 0 1];

ThetaOut = AllThetas;
end