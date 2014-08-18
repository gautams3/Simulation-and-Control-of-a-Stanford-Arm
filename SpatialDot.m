function [xySpatialDot] = SpatialDot(x,y)

SpatialDotIdentityMatrix = [zeros(3) eye(3); eye(3) zeros(3)];

xySpatialDot = x'*SpatialDotIdentityMatrix*y;
end