function [xSpatialCross] = SpatialCross(x)

omega = x(1:3);
mu = x(4:6); %here we assume v = [omega; mu] which is a line vector, not a free vector
xSpatialCross = zeros(6);

omegaCross = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];
muCross = [0 -mu(3) mu(2); mu(3) 0 -mu(1); -mu(2) mu(1) 0];

xSpatialCross = [omegaCross zeros(3); muCross omegaCross];
end