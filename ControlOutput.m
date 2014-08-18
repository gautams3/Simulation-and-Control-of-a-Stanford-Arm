function [FeedbackTorques] = ControlOutput(error, errorDot)

Kp = 3500;
Kd = 100;
Ki = 0;

FeedbackTorques = Kp*eye(6)*error + Kd*eye(6)*errorDot;
end