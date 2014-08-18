Simulation-and-Control-of-a-Stanford-Arm
========================================
Course project for ME384R: Advanced Dynamics of Robotic Systems

I have simulated the dynamics of a 6-axis Stanford arm in MATLAB, with a combination of feedforward and PID control to manipulate the Stanford arm and move an unknown payload. I have also implemented my own version of the Runge-Kutta 4th order solver with fixed timesteps. The file SimulationAndControlOfAStanfordArm.pdf has the complete description of the problem statement and data from my solution.

## Important modules:
### Inverse Kinematics (InverseKinematics.m)
This function takes the end effector position and orientation (in terms of a
rotation matrix) as inputs and outputs the possible branches of joint positions.

### Forward Kinematics
This function takes joint position values for each joint and outputs the
end effector position and orientation (rotation matrix).

### Task Planning
This module is used to calculate the desired position, velocity and acceleration
values, as well as the feed forward torques, offline. It uses the initial and final values of position,
velocity and acceleration of each joint to create a 5th order polynomial describing the desired
joint trajectory.

### Inverse Dynamics (InverseDynamics.m)
Inverse Dynamics calculates the torque required for a given state of the
Stanford arm. It takes the current dynamic state of the arm (position, velocity, acceleration) and
the payload mass to calculate the actual torque required to achieve this state.

### Forward Dynamics (FwdDynamics.m)
Calculation of forward dynamics involves calling the inverse dynamics
function several times. This function requires the current position, velocity and torque along with
the payload mass & geometry to give you the resulting acceleration.

### Controller(ControlOutput.m)
The controller is a simple PID control that takes the error and its derivative to
calculate the feedback torque. In the context of this simulation, simple PD control was effective
to give small errors.

## ODE Solver
This is a Runge Kutta rk45 fixed time solver I wrote.

## Main simulation file(main.m)
This file contains the main online simulation, and also some modules such
as task planning. It integrates all the modules and runs the simulation over the entire 10-sec
period of simulation.
