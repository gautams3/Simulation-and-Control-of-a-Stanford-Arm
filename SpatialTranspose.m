function [xSpatialTranspose] = SpatialTranspose(x)

% Takes the spatial transpose of a 6X6 matrix x
% If Y = [A B; C D] where A, B, C and D are 3x3 matrices
% Then Ys = [D' B'; C' A']

A = x(1:3,1:3);
B = x(1:3,4:6);
C = x(4:6,1:3);
D = x(4:6,4:6);

xSpatialTranspose = [D' B'; C' A'];
end