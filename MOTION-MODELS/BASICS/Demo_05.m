% Demo_05: Robot velocities to wheel velocities
%  Author: Ivan A. Calle Flores
clc; clearvars; close all

%----------------------------------------------------------------------%
% 1. INITIAL CONFIGURATION
%----------------------------------------------------------------------%
%  1.1. CREATE AN OBJECT "my_robot"
my_robot = robot();


%----------------------------------------------------------------------%
% 2. SET PARAMETERS
%----------------------------------------------------------------------%
%  2.1. MECHANICAL PARAMETERS
D = 2;          % Diameter of the wheels (m)
r = D/2;        % Radious
B = 1;          % Distance from the wheel to the center point (m)
my_robot.set_mechanical_parameters(D, B)
%  2.2. ROBOT VELOCITIES
v = 1.5;        % Translational velocity (rad/s)
w = 2.5;        % Rotational velocity (rad/s)


%----------------------------------------------------------------------%
% 3. COMPUTE THE ANGULAR VELOCICITIES OF THE MOTORS
%----------------------------------------------------------------------%
%  3.1. CALL THE METHOD THAT COMPUTE THE MOTOR VELOCITIES
[w_motor1, w_motor2] = my_robot.compute_motor_velocities(v,w);
%  3.2. DISPLAY THE RESULTS
disp('Angular velocities of the motors:')
disp([w_motor1  w_motor2] )
%  3.3. CHECK CORRECTNESS
vv = 0.5*r*(w_motor1 + w_motor2);
ww = 0.5*r/B*(w_motor1 - w_motor2);
disp('Check1:')
disp([vv  ww])
%  3.4. CHECK CORRECTNESS
[vv, ww] = my_robot.compute_robot_velocities(w_motor1,w_motor2);
disp('Check2:')
disp([vv  ww])