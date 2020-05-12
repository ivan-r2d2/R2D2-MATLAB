% Demo_04: Wheel velocities to robot velocities
%  Author: Ivan A. Calle Flores
clc; clearvars; close all

%----------------------------------------------------------------------%
% 1. INITIAL CONFIGURATION
%----------------------------------------------------------------------%
%  1.1. CREATE AN OBJECT "my_robot"
my_robot = robot();


%----------------------------------------------------------------------%
% 2. SET PARAMETERS OF THE ROBOT
%----------------------------------------------------------------------%
%  2.1. MECHANICAL PARAMETERS
D = 2;          % Diameter of the wheels (m)
B = 1;          % Distance from the wheel to the center point (m)
my_robot.set_mechanical_parameters(D,B)
%  2.2. VELOCITIES OF THE MOTORS
w_m1 = 3;       % Spinning speed of the right wheel "1" (rad/s)
w_m2 = -2;      % Spinning speed of the left wheel "2" (rad/s)


%----------------------------------------------------------------------%
% 3. GET THE VELOCICITIES IN THE LOCAL "REFERENCE" FRAME
%----------------------------------------------------------------------%
%  3.1. COMPUTE VVELOCITIES
[v,w] = my_robot.compute_robot_velocities(w_m1, w_m2);
%  3.2. DISPLAY RESULTS
x_r = [v; 0; w];  % [vx, vy, w]
disp('Velocities in the robot reference frame')
disp(x_r)