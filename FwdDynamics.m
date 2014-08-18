function [ThetaDoubleDot] = FwdDynamics(theta, thetaDot, Torque, msphere)
global MMatrix;

gravity = -9.82; %Accn due to gravity = -9.82 m/s2

thetan = theta;
thetadotn = thetaDot;

M = zeros(6);
FeedForwardTorque = Torque; % Feed fwd torque
Zero = zeros(6,1);
UnitVector = eye(6);

% bbar = V(theta,thetadot) + g(theta)
bbar = InverseDynamics(thetan, thetadotn, Zero, gravity, msphere);

for i = 1:6
M(:,i) = InverseDynamics(thetan, Zero, UnitVector(:,i), 0, msphere);
end

MMatrix = M;
ThetaDoubleDot = M\(FeedForwardTorque - bbar);
end