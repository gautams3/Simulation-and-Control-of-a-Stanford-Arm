function [answer] = StateDerivative(X, t)

global Torques;

theta = X(1:6);
thetadot = X(7:12);

answer = [thetadot ; FwdDynamics(theta, thetadot, Torques)];
end