function [answer] = SpringMassState(X, t)

%global Torques;

m = 100;
c = 0;
k = 10000;

y1 = X(1);
y2 = X(2);

y2Dot = (-c*y2 - k*y1)/m;

answer = [y2; y2Dot];
end