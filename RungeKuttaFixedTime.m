% Takes the current values of position and velocity and makes it into a
% 12x1 state vector [pos; vel]
% This vector is propagated through time using RungeKutta 4th order method

function [PosNext, VelNext] = RungeKuttaFixedTime(thetacurrent, Velcurrent, torque, msphere, timestep)
%For reference, FwdDynamics arguments are (theta, thetaDot, Torque, msphere)

    function [StateDerivative] = DerivativeFunction(State, torque, msphere)
    StateDerivative = [State(7:12); FwdDynamics(State(1:6) ,State(7:12), torque, msphere)];
    end

h = timestep;
CurrentState = [thetacurrent; Velcurrent];

k1 = h*DerivativeFunction(CurrentState, torque, msphere);

k2 = h*DerivativeFunction(CurrentState + k1/2, torque, msphere);

k3 = h*DerivativeFunction(CurrentState + k2/2, torque, msphere);

k4 = h*DerivativeFunction(CurrentState + k3, torque, msphere);

NextState = CurrentState + (k1 + 2*k2 + 2*k3 + k4)/6;

PosNext = NextState(1:6);
VelNext = NextState(7:12);
end