global l

Jacobian = zeros(6);

theta2 = ActualPos(end,2);
r3 = ActualPos(end,3);
theta4 = ActualPos(end,4);
theta5 = ActualPos(end,5);

Jacobian(1:3,1:3) = [sin(theta2) 0 0; 0 1 0; -cos(theta2) 0 0];
Jacobian(1:3,4:6) = [0 sin(theta4) cos(theta4)*cos(theta5); 0 -cos(theta4) sin(theta4)*sin(theta5); 1 0 -cos(theta5)];
Jacobian(4:6,1:3) = [l*cos(theta2) r3 0; -r3*sin(theta2) 0 0; l*sin(theta2) 0 1];

DeltaPos = Jacobian*Error(:,end);
EndEffectorPositionError = DeltaPos(4:6)